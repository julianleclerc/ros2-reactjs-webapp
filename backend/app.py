from flask import Flask, request, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS

import argparse
import os
import json
import time
import datetime

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "http://localhost:3000"}})
socketio = SocketIO(app, cors_allowed_origins="http://localhost:3000")

runtime_enabled = False
graphs = {}
ri_node = None  # TODO: ideally the ros node should not be exposed like that

### ACTION INTERFACE I/O

@app.route('/api/graphs/<key>', methods=['GET'])
def get_graph(key):
    value = graphs.get(key)
    if value:
        return jsonify(value)
    else:
        return jsonify({"error": "Data not found"}), 404

@app.route('/api/graphs/<key>', methods=['PUT'])
def set_graph(key):
    if key in graphs:
        new_data = request.get_json()
        graphs[key] = new_data
        print(f'Updated graph: {key}')
        return jsonify({"message": "Graph updated successfully"}), 200
    else:
        return jsonify({"error": "Graph not found"}), 404

@app.route('/api/graphs/exec/<key>', methods=['PUT'])
def exec_graph(key):
    if key in graphs:
        if "graph_state" not in graphs[key] or graphs[key]["graph_state"] != "RUNNING":
            ri_node.start_graph(key)
            print(f'Running graph "{key}"')
        elif graphs[key]["graph_state"] == "RUNNING":
            ri_node.stop_graph(key)
            print(f'Stopping graph "{key}"')

        return jsonify({"message": "Started graph"}), 200
    else:
        return jsonify({"error": "Graph not found"}), 404

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    socketio.emit('runtime_enabled', runtime_enabled)
    graphs_list = list(graphs.values())
    socketio.emit('graphs', graphs_list)

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

def load_graphs(graphs_dir):
    graphs = {}
    for filename in os.listdir(graphs_dir):
        if filename.endswith('.json'):
            with open(os.path.join(graphs_dir, filename), 'r') as file:
                graph = json.load(file)
                graphs[graph["graph_name"]] = graph
    return graphs

def graph_feedback_callback(actor, graphs_in):
    global graphs
    for g in graphs_in:
        g_json = json.loads(g)
        graphs[g_json["graph_name"]] = g_json
    graphs_list = list(graphs.values())
    socketio.emit('graphs', graphs_list)
    # save latest graph with actor name for planned actions
    umrf_store_feedback_data(actor,graphs_in)

### CHAT INTERFACE I/O

###### CHAT
chat_log = {}
request_response_tracking = {}

def get_current_timestamp():
    """Return current UTC time in ISO format."""
    return datetime.datetime.utcnow().isoformat() + "Z"

@app.route('/send_message', methods=['POST'])
def http_send_message():
    data = request.get_json()

    actor_list = data.get("actor")
    current_time = get_current_timestamp()
    user = "user"
    message = data.get("message")

    # Append the incoming message to each actor's log.
    for actor in actor_list:
        if actor not in chat_log:
            chat_log[actor] = []  # Initialize if not already present.
        chat_log[actor].append([current_time, user, message])

    # Check if ri_node is available and the message is not empty.
    if ri_node and message:
        ros_message = json.dumps({
            "targets": actor_list, 
            "message": message
        })
        
        ri_node.send_chat_message(ros_message)

        # Log a feedback message.
        current_time = get_current_timestamp()
        feedback_user = "debug"
        feedback_message = "Message sent"
        for actor in actor_list:
            chat_log[actor].append([current_time, feedback_user, feedback_message])
        
        socketio.emit('chat_log', chat_log)
        return jsonify({"debug": "Message sent"}), 200

    else:
        # Log an error message if sending failed.
        current_time = get_current_timestamp()
        error_user = "error"
        error_message = "Unable to send message"
        for actor in actor_list:
            chat_log[actor].append([current_time, error_user, error_message])

        socketio.emit('chat_log', chat_log)
        return jsonify({"error": "Unable to send message"}), 400

def chat_feedback_callback(message):
    """
    Callback function to handle chat messages from ROS.
    It parses the message, updates the chat_log, and emits the full log.
    """
    print(f"[DEBUG] In chat_feedback_callback with message: {message}")
    
    try:
        data = json.loads(message)

        actor_list = data.get("targets", [])
        current_time = get_current_timestamp()
        msg = data.get("message", "")
        msg_type = data.get("type", "")
        
        print(f"[DEBUG] Received message with type: {msg_type} for targets: {actor_list}")
        
        for actor in actor_list:
            # Use the actor's identifier as the user.
            user = str(actor)
            if actor not in chat_log:
                chat_log[actor] = []
            chat_log[actor].append([current_time, user, msg])
        
        socketio.emit('chat_log', chat_log)
        print("[DEBUG] Emitted chat_log event")
        
    except json.JSONDecodeError:
        print(f"[ERROR] Failed to parse JSON message: {message}")

@app.route('/add_new_actor', methods=['POST'])
def add_new_actor_to_log():
    data = request.get_json()
    actor_name = data.get("actor_name")

    if not actor_name:
        return jsonify({"error": "Missing actor_name"}), 400

    if actor_name in chat_log:
        return jsonify({"error": "Actor already exists"}), 400

    chat_log[actor_name] = []
    
    # Log a feedback message.
    current_time = get_current_timestamp()
    feedback_user = "debug"
    feedback_message = f"{actor_name} initialised"
    chat_log[actor_name].append([current_time, feedback_user, feedback_message])

    socketio.emit('chat_log', chat_log)
    return jsonify({"message": f"Actor {actor_name} added to chat log."}), 200

@app.route('/chat_interface_page_refresh', methods=['POST'])
def chat_interface_page_refresh():
    socketio.emit('chat_log', chat_log)
    socketio.emit('umrf_feedback_data', umrf_feedback_data)
    return jsonify({"message": "Chat interface refreshed", "chat_log": chat_log}), 200


###### PLANNED

umrf_feedback_data = {}

def umrf_store_feedback_data(actor,graph):
    umrf_feedback_data[actor] = graph
    socketio.emit('umrf_feedback_data', umrf_feedback_data)

### Server start time to save storage

SERVER_START_TIME = datetime.datetime.utcnow().isoformat() + "Z"

@app.route('/server_start_time', methods=['GET'])
def get_server_start_time():
    return jsonify({"server_start_time": SERVER_START_TIME}), 200


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', action='store_true', help='Enable run-time task monitoring.')
    args, unknown = parser.parse_known_args()

    runtime_enabled = args.runtime

    if runtime_enabled:
        import ros_interface as ri

        # Pass both the graph and chat feedback callbacks
        ri.run_ros_interface_thread(graph_feedback_callback, chat_feedback_callback)
        ri.wait_until_initialized()
        ri_node = ri.node

        graphs_indexed, graphs_running = ri.get_graphs()

        for g in graphs_indexed:
            g_json = json.loads(g)
            graphs[g_json["graph_name"]] = g_json

        print(" * ROS interface active")
    else:
        graphs = load_graphs("example_graphs")

    socketio.run(app, host='0.0.0.0', port=4000)
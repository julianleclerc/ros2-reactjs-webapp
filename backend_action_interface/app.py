from flask import Flask, request, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS

import argparse
import os
import json
import time

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "http://localhost:3000"}})
socketio = SocketIO(app, cors_allowed_origins="http://localhost:3000")

runtime_enabled = False
graphs = {}
actions = {}
ri_node = None # TODO: ideally the ros node should not be exposed like that

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
        print (f'Updated graph: {key}')
        # print (f'data   {json.dumps(new_data, indent=4)}')
        return jsonify({"message": "Graph updated successfully"}), 200
    else:
        return jsonify({"error": "Graph not found"}), 404

@app.route('/api/graphs/exec/<key>', methods=['PUT'])
def exec_graph(key):
    if key in graphs:
        if "graph_state" not in graphs[key] or graphs[key]["graph_state"] != "RUNNING":
            ri_node.start_graph(key)
            print (f'Running graph "{key}"')

        elif graphs[key]["graph_state"] == "RUNNING":
            ri_node.stop_graph(key)
            print (f'Stopping graph "{key}"')

        # graphs_list = list(graphs.values())
        # socketio.emit('graphs', graphs_list)

        return jsonify({"message": "Started graph"}), 200
    else:
        return jsonify({"error": "Graph not found"}), 404

@socketio.on('connect')
def handle_connect():
    print('Client connected')

    socketio.emit('runtime_enabled', runtime_enabled)

    graphs_list = list(graphs.values())
    socketio.emit('graphs', graphs_list)

    actions_list = list(actions.values())
    socketio.emit('actions', actions_list)

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

def load_actions(actions_dir):
    actions = {}
    for filename in os.listdir(actions_dir):
        if filename.endswith('.json'):
            with open(os.path.join(actions_dir, filename), 'r') as file:
                action = json.load(file)
                actions[action["name"]] = action
    return actions

def graph_feedback_callback(actor, graphs_in):
    global graphs
    global actions

    for g in graphs_in:
        g_json = json.loads(g)
        graphs[g_json["graph_name"]] = g_json

    graphs_list = list(graphs.values())
    socketio.emit('graphs', graphs_list)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', action='store_true', help='Enable run-time task monitoring.')
    args, unknown = parser.parse_known_args()

    runtime_enabled = args.runtime

    if runtime_enabled:
        import ros_interface as ri

        ri.run_ros_interface_thread(graph_feedback_callback)
        ri.wait_until_initialized()
        ri_node = ri.node

        actions_list, graphs_indexed, graphs_running = ri.get_graphs()
        for a in actions_list:
            a_json = json.loads(a)
            actions[a_json["name"]] = a_json

        for g in graphs_indexed:
            g_json = json.loads(g)
            graphs[g_json["graph_name"]] = g_json

        print (" * ROS interface active")
    else:
        graphs = load_graphs("example_graphs")
        actions = load_actions("example_actions")

    socketio.run(app, host='0.0.0.0', port=4000)

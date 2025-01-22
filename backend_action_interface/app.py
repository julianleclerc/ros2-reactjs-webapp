from flask import Flask, request, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS
import threading
import time
import os
import json

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "http://localhost:3000"}})
socketio = SocketIO(app, cors_allowed_origins="http://localhost:3000")

# Directory containing the JSON files
GRAPHS_DIR = 'example_graphs'

def load_graphs():
    graphs = {}
    for filename in os.listdir(GRAPHS_DIR):
        if filename.endswith('.json'):
            with open(os.path.join(GRAPHS_DIR, filename), 'r') as file:
                graph = json.load(file)
                graphs[graph["graph_name"]] = graph
    return graphs

graphs = load_graphs()
graph_names = [key for key in graphs]

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
        print ("no jesbos")
        return jsonify({"error": "Graph not found"}), 404

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    socketio.emit('graphNames', graph_names)

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

# def add_item():
#     time.sleep(5)
#     new_graph = {"name": "graph_name_3.json", "status": "active", "data": { /* JSON data for graph_name_3 */ }}
#     items.append(new_graph)
#     socketio.emit('updateItems', items)

if __name__ == '__main__':
    # # Start the background thread that adds an item after 5 seconds
    # threading.Thread(target=add_item).start()
    socketio.run(app, host='0.0.0.0', port=4000)

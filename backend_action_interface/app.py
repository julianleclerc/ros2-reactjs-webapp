from flask import Flask, request, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS

import argparse
import os
import json
import rclpy
from temoto_msgs.srv import UmrfGraphGet


app = Flask(__name__)
app.config['DEBUG'] = True

CORS(app, resources={r"/*": {"origins": "http://localhost:3000"}})
socketio = SocketIO(app, cors_allowed_origins="http://localhost:3000")

runtime_enabled = False
graphs = {}
ri_node = None # TODO: ideally the ros node should not be exposed like that

def load_graphs(graphs_dir):
    graphs = {}
    for filename in os.listdir(graphs_dir):
        if filename.endswith('.json'):
            with open(os.path.join(graphs_dir, filename), 'r') as file:
                graph = json.load(file)
                graphs[graph["graph_name"]] = graph
    return graphs

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

        graphs_list = list(graphs.values())
        socketio.emit('graphs', graphs_list)

        return jsonify({"message": "Graph updated successfully"}), 200
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

# def add_item():
#     time.sleep(5)
#     new_graph = {"name": "graph_name_3.json", "status": "active", "data": { /* JSON data for graph_name_3 */ }}
#     items.append(new_graph)
#     socketio.emit('updateItems', items)


def get_umrf_graphs():
    rclpy.init()
    node = rclpy.create_node('react_service_client')

    client = node.create_client(UmrfGraphGet, 'umrf_graph_get')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    # Create a request for the UmrfGraphGet service
    request = UmrfGraphGet.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)    
    result = future.result()

    node.destroy_node()
    rclpy.shutdown()

    if result:
        # Return the actual vectors from the service response
        return {
            "graph_jsons_indexed": result.graph_jsons_indexed,
            "graph_jsons_running": result.graph_jsons_running
        }
    else:
        return None  # Return None if the service call was not successful

@app.route('/get-umrf-graphs-ros-service', methods=['POST'])
def get_umrf_graphs_service():
    try:
        app.logger.info('Received POST request on /get-umrf-graphs-ros-service')
        result_data = get_umrf_graphs()  # Call ROS2 service
        
        if result_data:
            app.logger.info('Service call successful')
            # Return the result data (graph_jsons_indexed, graph_jsons_running)
            return jsonify(result_data), 200
        else:
            app.logger.error('Service call failed')
            return jsonify({'message': 'Service call failed!'}), 500
    except Exception as e:
        app.logger.error(f'Error: {e}')
        return jsonify({'message': 'An error occurred: ' + str(e)}), 500




if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', action='store_true', help='Enable run-time task monitoring.')
    args, unknown = parser.parse_known_args()

    runtime_enabled = args.runtime

    if runtime_enabled:
        import ros_interface as ri

        ri.run_ros_interface_thread()
        ri.wait_until_initialized()
        ri_node = ri.node

        graphs_indexed, graphs_running = ri.get_graphs()

        for g in graphs_indexed:
            g_json = json.loads(g)
            graphs[g_json["graph_name"]] = g_json

        print (" * ROS interface active")
    else:
        graphs = load_graphs("example_graphs")

    socketio.run(app, host='0.0.0.0', port=4000)

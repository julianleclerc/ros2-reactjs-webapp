# app.py

from flask import Flask, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO
from threading import Thread
import rclpy
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime

from ros_node import ROS2Node
from routes import init_routes
import config

def create_app():
    app = Flask(__name__)
    CORS(app, resources={r"/*": {"origins": "*"}})
    socketio = SocketIO(app, cors_allowed_origins="*")

    # Generate server start time
    app.server_start_time = datetime.utcnow().isoformat()
    print("Server start time:", app.server_start_time)

    # Initialize ROS2
    rclpy.init()
    ros2_node = ROS2Node(socketio)
    executor = MultiThreadedExecutor()
    executor.add_node(ros2_node)

    def ros2_spin():
        try:
            executor.spin()
        finally:
            executor.shutdown()
            ros2_node.destroy_node()
            rclpy.shutdown()

    ros2_thread = Thread(target=ros2_spin, daemon=True)
    ros2_thread.start()

    # Initialize application routes
    init_routes(app, ros2_node, socketio)

    return app, socketio

app, socketio = create_app()

@app.route('/server_start_time')
def get_server_start_time():
    return jsonify({'server_start_time': app.server_start_time})

if __name__ == '__main__':
    socketio.run(app, host=config.FLASK_HOST, port=config.FLASK_PORT)
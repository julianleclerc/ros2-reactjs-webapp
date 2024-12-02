# backend/app.py

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
from threading import Thread
import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros_node import ROS2Node
from routes import init_routes
import config

def create_app():
    app = Flask(__name__)
    CORS(app)
    socketio = SocketIO(app, cors_allowed_origins="*")

    # Initialize ROS2
    rclpy.init()
    ros2_node = ROS2Node(socketio)

    # Create executor and add the node
    executor = MultiThreadedExecutor()
    executor.add_node(ros2_node)

    def ros2_spin():
        try:
            executor.spin()
        finally:
            executor.shutdown()
            ros2_node.destroy_node()
            rclpy.shutdown()

    # Start ROS2 spinning in a separate thread
    ros2_thread = Thread(target=ros2_spin)
    ros2_thread.start()

    # Initialize routes
    init_routes(app, ros2_node)

    return app, socketio

app, socketio = create_app()

if __name__ == '__main__':
    socketio.run(app, host=config.FLASK_HOST, port=config.FLASK_PORT)

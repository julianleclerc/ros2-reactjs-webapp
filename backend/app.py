# app.py

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
    """
    Create and configure the Flask app, initialize ROS2, and set up routes and SocketIO.
    """
    # Initialize Flask app with CORS and SocketIO
    app = Flask(__name__)
    CORS(app, resources={r"/*": {"origins": "*"}})
    socketio = SocketIO(app, cors_allowed_origins="*")

    # Initialize ROS2
    rclpy.init()
    ros2_node = ROS2Node(socketio)

    # Create and configure the ROS2 executor
    executor = MultiThreadedExecutor()
    executor.add_node(ros2_node)

    def ros2_spin():
        """
        Spin the ROS2 executor in a separate thread.
        """
        try:
            executor.spin()
        finally:
            executor.shutdown()
            ros2_node.destroy_node()
            rclpy.shutdown()

    # Start ROS2 spinning in a separate thread
    ros2_thread = Thread(target=ros2_spin, daemon=True)
    ros2_thread.start()

    # Initialize application routes
    init_routes(app, ros2_node, socketio)

    return app, socketio

# Create Flask app and SocketIO instance
app, socketio = create_app()

if __name__ == '__main__':
    # Run the Flask app with SocketIO
    socketio.run(app, host=config.FLASK_HOST, port=config.FLASK_PORT)

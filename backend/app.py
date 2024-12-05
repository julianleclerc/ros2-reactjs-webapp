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

# ----------------- Application Factory -----------------

def create_app():
    """
    Creates and configures the Flask application, initializes ROS2 and routes, and starts the ROS2 spinning thread.
    """
    # Create the Flask app and enable CORS
    app = Flask(__name__)
    CORS(app, resources={r"/*": {"origins": "*"}})

    # Initialize Flask-SocketIO
    socketio = SocketIO(app, cors_allowed_origins="*")

    # Generate and store server start time
    app.server_start_time = datetime.utcnow().isoformat()
    print("Server start time:", app.server_start_time)

    # Initialize ROS2 and its executor
    rclpy.init()
    ros2_node = ROS2Node(socketio)
    executor = MultiThreadedExecutor()
    executor.add_node(ros2_node)

    # Define ROS2 spinning thread
    def ros2_spin():
        try:
            executor.spin()
        finally:
            executor.shutdown()
            ros2_node.destroy_node()
            rclpy.shutdown()

    # Start ROS2 spinning thread
    ros2_thread = Thread(target=ros2_spin, daemon=True)
    ros2_thread.start()

    # Initialize application routes
    init_routes(app, ros2_node, socketio)

    return app, socketio

# ----------------- Application and SocketIO Instance -----------------

# Create the Flask app and Socket.IO instance
app, socketio = create_app()

# ----------------- Flask Routes -----------------

@app.route('/server_start_time')
def get_server_start_time():
    """
    Returns the server start time in ISO format.
    """
    return jsonify({'server_start_time': app.server_start_time})

# ----------------- Application Entry Point -----------------

if __name__ == '__main__':
    # Run the Flask app with Socket.IO
    socketio.run(app, host=config.FLASK_HOST, port=config.FLASK_PORT)

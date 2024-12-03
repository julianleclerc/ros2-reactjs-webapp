# ros_node.py

import threading
import time
import json
import cv2
import base64
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from interfaces.srv import Chat

class ROS2Node(Node):
    """
    ROS2 Node to interact with Flask backend and handle ROS2 topics and services.
    """

    def __init__(self, socketio):
        super().__init__('flask_ros2_node')
        self.socketio = socketio

        # Initialize the CvBridge for image conversion
        self.bridge = CvBridge()

        # --- Publishers ---
        self.prompt_response_publisher = self.create_publisher(String, '/prompt_response', 10)

        # --- Subscribers for ChatPanel ---
        self.prompt_request_subscription = self.create_subscription(
            String, '/prompt_request', self.prompt_request_callback, 10)
        self.prompt_alert_subscription = self.create_subscription(
            String, '/prompt_alert', self.prompt_alert_callback, 10)
        self.prompt_info_subscription = self.create_subscription(
            String, '/prompt_info', self.prompt_info_callback, 10)

        # --- Subscribers for PlannedActionPanel ---
        self.queue_subscription = self.create_subscription(
            String, '/json_queue', self.queue_callback, 10)
        self.action_status_subscription = self.create_subscription(
            String, '/action_status', self.action_status_callback, 10)

        # --- Subscribers for DisplayPanel ---
        self.camera_subscription = self.create_subscription(
            Image, 'camera/image', self.camera_callback, 10)
        self.rviz_subscription = self.create_subscription(
            Image, 'rviz_display_base64', self.rviz_callback, 10)

        # --- Service Client ---
        self.chat_request_client = self.create_client(Chat, 'chat_service')

        # Wait for the chat service to be available
        self._wait_for_service()

        # --- Flags and Locks ---
        self.next_message_to_prompt_response = False
        self.flag_lock = threading.Lock()

        # --- Latest Data ---
        self.latest_queue = []
        self.latest_action_status = ''

    def _wait_for_service(self):
        """
        Wait for the chat_service to be available.
        """
        while not self.chat_request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for chat_service service...')
            time.sleep(1)
        self.get_logger().info('Connected to chat_service service.')

    # --- ChatPanel Callbacks ---

    def prompt_request_callback(self, msg):
        """
        Handle messages from the /prompt_request topic.
        """
        self.socketio.emit('ros_message', {'user': 'ROS', 'message': msg.data, 'type': 'default'})
        self.get_logger().info(f'Received prompt_request: {msg.data}')
        with self.flag_lock:
            self.next_message_to_prompt_response = True
        self.get_logger().info('Next user message will be sent to /prompt_response')

    def prompt_alert_callback(self, msg):
        """
        Handle messages from the /prompt_alert topic.
        """
        self.socketio.emit('ros_message', {'user': 'ROS', 'message': msg.data, 'type': 'alert'})
        self.get_logger().info(f'Received prompt_alert: {msg.data}')

    def prompt_info_callback(self, msg):
        """
        Handle messages from the /prompt_info topic.
        """
        self.socketio.emit('ros_message', {'user': 'ROS', 'message': msg.data, 'type': 'info'})
        self.get_logger().info(f'Received prompt_info: {msg.data}')

    # --- PlannedActionPanel Callbacks ---

    def queue_callback(self, msg):
        """
        Handle messages from the /json_queue topic.
        """
        self.get_logger().info(f'Received /json_queue message: {msg.data}')
        try:
            data = json.loads(msg.data)
            queue = data.get('queue', [])
            self.latest_queue = queue
            self.socketio.emit('queue_update', {'queue': queue})
            self.get_logger().info('Emitted queue_update to frontend.')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse /json_queue message: {e}')

    def action_status_callback(self, msg):
        """
        Handle messages from the /action_status topic.
        """
        status = msg.data.strip().lower()
        self.latest_action_status = status
        self.socketio.emit('action_status', {'status': status})
        self.get_logger().info(f'Received /action_status message: {status}')
        self.get_logger().info('Emitted action_status to frontend.')

    # --- DisplayPanel Callbacks ---

    def camera_callback(self, msg):
        """
        Handle messages from the camera/image topic.
        Converts ROS image to base64 and emits to the frontend.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            self.socketio.emit('camera_image', image_base64, room='camera')
            self.get_logger().info('Emitted camera image to frontend.')
        except Exception as e:
            self.get_logger().error(f'Failed to process camera image: {e}')

    def rviz_callback(self, msg):
        """
        Handle messages from the rviz_display_base64 topic.
        Converts ROS image to base64 and emits to the frontend.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            self.socketio.emit('rviz_image', image_base64, room='rviz')
            self.get_logger().info('Emitted rviz image to frontend.')
        except Exception as e:
            self.get_logger().error(f'Failed to process rviz image: {e}')

    # --- Service Call ---

    def call_chat_service(self, message):
        """
        Call the chat_service with the given message and return the response.
        """
        request = Chat.Request()
        request.message = message
        future = self.chat_request_client.call_async(request)

        while not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            self.get_logger().info('Received response from chat_service.')
            return future.result().response
        else:
            self.get_logger().error('Service call to chat_service failed.')
            return None

    # --- Publisher Method ---

    def publish_prompt_response(self, message):
        """
        Publish a message to the /prompt_response topic.
        """
        msg = String()
        msg.data = message
        self.prompt_response_publisher.publish(msg)
        self.get_logger().info(f'Published to /prompt_response: {message}')

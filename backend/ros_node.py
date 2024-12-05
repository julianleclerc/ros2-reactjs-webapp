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

        # --- Initial Setup ---
        self.bridge = CvBridge()
        self.next_message_to_prompt_response = False
        self.flag_lock = threading.Lock()
        self.latest_queue = []
        self.latest_action_status = ''
        self.latest_camera_image = None

        # --- Publishers ---
        self.prompt_response_publisher = self.create_publisher(String, '/prompt_response', 10)

        # --- Subscribers ---
        self.prompt_request_subscription = self.create_subscription(
            String, '/prompt_request', self.prompt_request_callback, 10)
        self.queue_subscription = self.create_subscription(
            String, '/json_queue', self.queue_callback, 10)
        self.action_status_subscription = self.create_subscription(
            String, '/action_status', self.action_status_callback, 10)
        self.camera_subscription = self.create_subscription(
            Image, 'camera/image', self.camera_callback, 10)

        # --- Service Client ---
        self.chat_request_client = self.create_client(Chat, 'chat_service')
        self._wait_for_service()

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

    # --- PlannedActionPanel Callbacks ---

    def queue_callback(self, msg):
        """
        Handle messages from the /json_queue topic.
        """
        try:
            data = json.loads(msg.data)
            self.latest_queue = data.get('queue', [])
            self.socketio.emit('queue_update', {'queue': self.latest_queue})
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse /json_queue message: {e}')

    def action_status_callback(self, msg):
        """
        Handle messages from the /action_status topic.
        """
        self.latest_action_status = msg.data.strip().lower()
        self.socketio.emit('action_status', {'status': self.latest_action_status})

    # --- DisplayPanel Callbacks ---

    def camera_callback(self, msg):
        """
        Handle messages from the camera/image topic.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            self.latest_camera_image = base64.b64encode(buffer).decode('utf-8')
            self.socketio.emit('camera_image', self.latest_camera_image, room='camera')
        except Exception as e:
            self.get_logger().error(f'Failed to process camera image: {e}')

    # --- Service Calls ---

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
            return future.result().response
        else:
            return None

    # --- Publisher Methods ---

    def publish_prompt_response(self, message):
        """
        Publish a message to the /prompt_response topic.
        """
        msg = String()
        msg.data = message
        self.prompt_response_publisher.publish(msg)

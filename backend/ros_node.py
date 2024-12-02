# backend/ros_node.py

import threading
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.srv import Chat
from rclpy.executors import MultiThreadedExecutor

class ROS2Node(Node):
    def __init__(self, socketio):
        super().__init__('flask_ros2_node')
        self.socketio = socketio

        # Publishers
        self.publisher = self.create_publisher(String, '/prompt_response', 10)

        # Subscribers for ChatPanel
        self.prompt_request_subscription = self.create_subscription(
            String,
            '/prompt_request',
            self.prompt_request_callback,
            10)
        self.prompt_alert_subscription = self.create_subscription(
            String,
            '/prompt_alert',
            self.prompt_alert_callback,
            10)
        self.prompt_info_subscription = self.create_subscription(
            String,
            '/prompt_info',
            self.prompt_info_callback,
            10)

        # Subscribers for PlannedActionPanel
        self.queue_subscription = self.create_subscription(
            String,
            '/json_queue',
            self.queue_callback,
            10)
        self.action_status_subscription = self.create_subscription(
            String,
            '/action_status',
            self.action_status_callback,
            10)

        # Service client
        self.chat_request_client = self.create_client(Chat, 'chat_service')

        # Wait for the service to be available
        while not self.chat_request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for chat_service service...')
            time.sleep(1)

        self.get_logger().info('Connected to chat_service service.')

        # Flag and lock for prompt responses
        self.next_message_to_prompt_response = False
        self.flag_lock = threading.Lock()

        # Store latest data
        self.latest_queue = []
        self.latest_action_status = ''

    # --- ChatPanel Callbacks ---

    def prompt_request_callback(self, msg):
        self.socketio.emit('ros_message', {
            'user': 'ROS',
            'message': msg.data,
            'type': 'default'
        })
        self.get_logger().info(f'Received prompt_request: {msg.data}')

        with self.flag_lock:
            self.next_message_to_prompt_response = True
        self.get_logger().info('Next user message will be sent to /prompt_response')

    def prompt_alert_callback(self, msg):
        self.socketio.emit('ros_message', {
            'user': 'ROS',
            'message': msg.data,
            'type': 'alert'
        })
        self.get_logger().info(f'Received prompt_alert: {msg.data}')

    def prompt_info_callback(self, msg):
        self.socketio.emit('ros_message', {
            'user': 'ROS',
            'message': msg.data,
            'type': 'info'
        })
        self.get_logger().info(f'Received prompt_info: {msg.data}')

    # --- PlannedActionPanel Callbacks ---

    def queue_callback(self, msg):
        self.get_logger().info(f'Received /queue message: {msg.data}')
        try:
            data = json.loads(msg.data)
            queue = data.get('queue', [])
            self.latest_queue = queue
            self.socketio.emit('queue_update', {'queue': queue})
            self.get_logger().info('Emitted queue_update to frontend.')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse /queue message: {e}')

    def action_status_callback(self, msg):
        status = msg.data.strip().lower()
        self.latest_action_status = status
        self.get_logger().info(f'Received /action_status message: {status}')
        self.socketio.emit('action_status', {'status': status})
        self.get_logger().info('Emitted action_status to frontend.')

    # --- Service Call ---

    def call_chat_service(self, message):
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
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published to /prompt_response: {message}')

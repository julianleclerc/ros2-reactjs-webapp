import rclpy
from rclpy.node import Node
import json
import threading
import time

from temoto_msgs.srv import UmrfGraphGet
from temoto_msgs.msg import UmrfGraphStart, UmrfGraphStop, UmrfGraphFeedback
from std_msgs.msg import String

node = None

def wait_until_initialized():
    while node is None:
        time.sleep(1)

def get_graphs():
    if node is None:
        return None
    return node.get_graphs()

class RosInterface(Node):
    """A ROS2 Node that interacts with both the action and chat interfaces."""
    def __init__(self, graph_feedback_cb_parent, chat_feedback_cb_parent):
        super().__init__('action_designer_runtime')
        self.graph_feedback_cb_parent = graph_feedback_cb_parent
        self.chat_feedback_cb_parent = chat_feedback_cb_parent

        ### ACTION INTERFACE TOPICS
        self.umrf_graph_start_pub = self.create_publisher(UmrfGraphStart, 'umrf_graph_start', 10)
        self.umrf_graph_stop_pub = self.create_publisher(UmrfGraphStop, 'umrf_graph_stop', 10)
        self.umrf_graph_feedback_sub = self.create_subscription(
            UmrfGraphFeedback, 'umrf_graph_feedback', self.umrf_graph_feedback_cb, 10)
        self.client_graph_get = self.create_client(UmrfGraphGet, 'umrf_graph_get')

        ### CHAT INTERFACE TOPICS
        self.chat_interface_user_input_pub = self.create_publisher(String, 'chat_interface_user_input', 10)
        self.chat_interface_feedback_sub = self.create_subscription(
            String, 'chat_interface_feedback', self.handle_chat_interface_feedback, 10)

        while not self.client_graph_get.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for the umrf_graph_get server')
        self.get_logger().info('umrf_graph_get server is up')

    ### Action Interface Methods
    def get_graphs(self):
        req = UmrfGraphGet.Request()
        res = self.client_graph_get.call(req)
        return res.graph_jsons_indexed, res.graph_jsons_running

    def start_graph(self, graph_name):
        msg = UmrfGraphStart()
        msg.umrf_graph_name = graph_name
        msg.name_match_required = False
        msg.targets = ['David']
        self.umrf_graph_start_pub.publish(msg)

    def stop_graph(self, graph_name):
        msg = UmrfGraphStop()
        msg.umrf_graph_name = graph_name
        msg.targets = ['David']
        self.umrf_graph_stop_pub.publish(msg)

    def umrf_graph_feedback_cb(self, msg):
        self.graph_feedback_cb_parent(msg.actor, msg.history)

    ### Chat Interface Methods
    def send_chat_message(self, msg):
        self.get_logger().info(f"[DEBUG] handle_chat_input: Received chat input: {msg}")
        response_msg = String()
        response_msg.data = msg
        self.chat_interface_user_input_pub.publish(response_msg)

    def handle_chat_interface_feedback(self, msg):
        self.get_logger().info(f"[DEBUG] handle_chat_interface_feedback: Received chat input: {msg}")
        data = msg.data
        # Forward the received chat message to the Flask/Socket.IO callback.
        self.chat_feedback_cb_parent(data)

def run_ros_interface(graph_feedback_callback, chat_feedback_callback):
    global node
    rclpy.init()
    node = RosInterface(graph_feedback_callback, chat_feedback_callback)
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except rclpy.executors.ExternalShutdownException:
        node.get_logger().info("External shutdown signal received, stopping ROS2 node.")

def run_ros_interface_thread(graph_feedback_callback, chat_feedback_callback):
    thread = threading.Thread(target=run_ros_interface, args=(graph_feedback_callback, chat_feedback_callback))
    thread.start()

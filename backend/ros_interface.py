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
        
        # Track actors with pending requests that need responses
        self.actors_awaiting_response = set()

        ### ACTION INTERFACE TOPICS
        self.umrf_graph_start_pub = self.create_publisher(UmrfGraphStart, 'umrf_graph_start', 10)
        self.umrf_graph_stop_pub = self.create_publisher(UmrfGraphStop, 'umrf_graph_stop', 10)
        self.umrf_graph_feedback_sub = self.create_subscription(
            UmrfGraphFeedback, 'umrf_graph_feedback', self.umrf_graph_feedback_cb, 10)
        self.client_graph_get = self.create_client(UmrfGraphGet, 'umrf_graph_get')

        ### CHAT INTERFACE TOPICS
        self.chat_interface_input_pub = self.create_publisher(String, 'chat_interface_input', 10)
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
        self.get_logger().info(f"[-----> DEBUG] msg.history: {msg.history}")
        self.graph_feedback_cb_parent(msg.actor, msg.history)

    ### Chat Interface Methods
    def send_chat_message(self, msg_str):
        self.get_logger().info(f"[DEBUG] handle_chat_input: Received chat input: {msg_str}")
        
        # Parse the message
        try:
            msg_data = json.loads(msg_str)
            targets = msg_data.get("targets", [])
            message = msg_data.get("message", "")
            
            # Check if any targets are awaiting a response
            # If they are, set the type to "response", otherwise use "request"
            awaiting_response = any(target in self.actors_awaiting_response for target in targets)
            
            if awaiting_response:
                # Set message type to "response" for actors awaiting a response
                msg_data["type"] = "response"
                # Remove the actors from the awaiting_response set
                for target in targets:
                    if target in self.actors_awaiting_response:
                        self.actors_awaiting_response.remove(target)
            else:
                # Set message type to "request" by default
                msg_data["type"] = "request"
                
            # Convert back to JSON string
            updated_msg_str = json.dumps(msg_data)
            
            # Publish the message
            response_msg = String()
            response_msg.data = updated_msg_str
            self.chat_interface_input_pub.publish(response_msg)
            
            self.get_logger().info(f"[DEBUG] Published message with type: {msg_data['type']}")
            
        except json.JSONDecodeError:
            self.get_logger().error(f"[ERROR] Failed to parse JSON message: {msg_str}")
            # Fallback to publishing the original message
            response_msg = String()
            response_msg.data = msg_str
            self.chat_interface_input_pub.publish(response_msg)

    def handle_chat_interface_feedback(self, msg):
        self.get_logger().info(f"[DEBUG] handle_chat_interface_feedback: Received chat input: {msg}")
        data = msg.data
        
        try:
            # Parse the feedback message
            feedback_data = json.loads(data)
            message_type = feedback_data.get("type", "")
            targets = feedback_data.get("targets", [])
            
            # If the message type is "request", add the targets to the awaiting_response set
            if message_type == "request":
                for target in targets:
                    self.actors_awaiting_response.add(target)
                self.get_logger().info(f"[DEBUG] Added targets to awaiting_response: {targets}")
            
            # If the message type is "info" or "error", remove the targets from the awaiting_response set
            elif message_type in ["info", "error"]:
                for target in targets:
                    if target in self.actors_awaiting_response:
                        self.actors_awaiting_response.remove(target)
                self.get_logger().info(f"[DEBUG] Removed targets from awaiting_response due to {message_type}: {targets}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"[ERROR] Failed to parse JSON feedback message: {data}")
        
        # Forward the received chat message to the Flask/Socket.IO callback
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
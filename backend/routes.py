from flask import request, jsonify
from flask_socketio import emit, join_room, leave_room

def init_routes(app, ros2_node, socketio):
    """
    Initialize Flask routes and Socket.IO events with the given app, ROS2 node, and Socket.IO instance.
    """

    # ----------------- Flask Routes -----------------

    @app.route('/send_message', methods=['POST'])
    def send_message():
        """
        Endpoint to handle message sending to ROS2.

        Request JSON:
        - message: The message to send.

        Responses:
        - 200: Success with a response message or an empty response if prompt_response was handled.
        - 500: Error when ROS2 node is not initialized or response fails.
        """
        try: 
            data = request.json
            message = data.get('message', '')

            # Check if ROS2 node is initialized
            if ros2_node is None:
                return jsonify({'error': 'ROS2 node is not initialized'}), 500

            # Use a lock to safely handle ROS2 node interactions
            with ros2_node.flag_lock:
                # Publish to /prompt_response if the flag is set
                if ros2_node.next_message_to_prompt_response:
                    ros2_node.publish_prompt_response(message)
                    ros2_node.next_message_to_prompt_response = False
                    # Return an empty JSON to indicate success but no message for chat
                    return jsonify({}), 200

                # Call the chat service for normal messages
                response = ros2_node.call_chat_service(message)
                if response:
                    return jsonify({'response': response})
                else:
                    return jsonify({'error': 'Failed to get response from ROS2'}), 500
        except Exception as e:
            return jsonify({'error': 'Internal server error', 'details': str(e)}), 500

    @app.route('/current_queue', methods=['GET'])
    def get_current_queue():
        """
        Fetch the latest action queue.
        """
        return jsonify({'queue': ros2_node.latest_queue})

    @app.route('/current_action_status', methods=['GET'])
    def get_current_action_status():
        """
        Fetch the latest action status.
        """
        return jsonify({'status': ros2_node.latest_action_status})

    # ----------------- Socket.IO Events -----------------

    @socketio.on('connect')
    def handle_connect():
        """
        Handles new client connections.
        Logs the client's session ID.
        """
        ros2_node.get_logger().info(f'Client connected: {request.sid}')

    @socketio.on('disconnect')
    def handle_disconnect():
        """
        Handles client disconnections.
        Logs the client's session ID and removes them from any rooms.
        """
        ros2_node.get_logger().info(f'Client disconnected: {request.sid}')
        leave_room('camera', sid=request.sid, namespace='/')
        leave_room('rviz', sid=request.sid, namespace='/')

    @socketio.on('subscribe_to_camera')
    def handle_subscribe_camera():
        """
        Handles client subscriptions to the camera feed.
        Adds the client to the 'camera' room.
        """
        ros2_node.get_logger().info(f'Client {request.sid} subscribed to camera')
        join_room('camera', sid=request.sid, namespace='/')

    @socketio.on('unsubscribe_from_camera')
    def handle_unsubscribe_camera():
        """
        Handles client unsubscriptions from the camera feed.
        Removes the client from the 'camera' room.
        """
        ros2_node.get_logger().info(f'Client {request.sid} unsubscribed from camera')
        leave_room('camera', sid=request.sid, namespace='/')

    @socketio.on('subscribe_to_rviz')
    def handle_subscribe_rviz():
        """
        Handles client subscriptions to the RViz feed.
        Adds the client to the 'rviz' room.
        """
        ros2_node.get_logger().info(f'Client {request.sid} subscribed to rviz')
        join_room('rviz', sid=request.sid, namespace='/')

    @socketio.on('unsubscribe_from_rviz')
    def handle_unsubscribe_rviz():
        """
        Handles client unsubscriptions from the RViz feed.
        Removes the client from the 'rviz' room.
        """
        ros2_node.get_logger().info(f'Client {request.sid} unsubscribed from rviz')
        leave_room('rviz', sid=request.sid, namespace='/')

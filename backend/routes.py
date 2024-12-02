from flask import request, jsonify

def init_routes(app, ros2_node):
    """
    Initialize Flask routes with the given app and ROS2 node.
    """
    @app.route('/send_message', methods=['POST'])
    def send_message():
        """
        Endpoint to handle message sending to ROS2.

        Request JSON:
        - message: The message to send.

        Responses:
        - 200: Success with a response message.
        - 500: Error when ROS2 node is not initialized or response fails.
        """
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
                return jsonify({'response': 'Message sent to /prompt_response'})

            # Call the chat service for normal messages
            response = ros2_node.call_chat_service(message)
            if response:
                return jsonify({'response': response})
            else:
                return jsonify({'error': 'Failed to get response from ROS2'}), 500

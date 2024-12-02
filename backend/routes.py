# backend/routes.py

from flask import request, jsonify

def init_routes(app, ros2_node):
    @app.route('/send_message', methods=['POST'])
    def send_message():
        data = request.json
        message = data.get('message', '')

        if ros2_node is None:
            return jsonify({'error': 'ROS2 node is not initialized'}), 500

        with ros2_node.flag_lock:
            if ros2_node.next_message_to_prompt_response:
                ros2_node.publish_prompt_response(message)
                ros2_node.next_message_to_prompt_response = False
                response_message = 'Message sent to /prompt_response'
                return jsonify({'response': response_message})
            else:
                response = ros2_node.call_chat_service(message)
                if response:
                    return jsonify({'response': response})
                else:
                    return jsonify({'error': 'Failed to get response from ROS2'}), 500

    # Additional routes can be added here

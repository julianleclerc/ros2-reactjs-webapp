from flask import Blueprint, request, jsonify
from ..ros.ros_services import chat_service

chat_bp = Blueprint('chat', __name__)

@chat_bp.route('/api/chat', methods=['POST'])
def chat_handler():
    data = request.json
    chat_request = {'message': data['message']}
    result = chat_service.call_sync(chat_request)
    return jsonify({'result': result})

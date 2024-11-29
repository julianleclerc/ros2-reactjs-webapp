from flask import Blueprint, jsonify

queue_bp = Blueprint('queue', __name__)

@queue_bp.route('/api/queue', methods=['GET'])
def get_queue():
    return jsonify({"queue": ["action1", "action2", "action3"]})

from roslibpy import Service
from .ros_client import ros_client

chat_service = Service(ros_client, '/chat_service', 'interfaces/srv/Chat')

import roslibpy

ros_client = roslibpy.Ros(host='localhost', port=9090)
ros_client.run()

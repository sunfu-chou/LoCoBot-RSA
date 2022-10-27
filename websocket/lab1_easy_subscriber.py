from arg_robotics_tools import websocket_rosbridge as socket

def subscriber_callback(message):
    print(message)

easy_subscriber = socket.ros_socket('0.0.0.0', 9090)

easy_subscriber.subscriber('/subscribe_topic', subscriber_callback , 1000)

while True:
    pass
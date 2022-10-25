from arg_robotics_tools import websocket_rosbridge as socket

import time
easy_publisher = socket.ros_socket('0.0.0.0', 9090)

data = {'data':'this is a test string.'}
talker = easy_publisher.publisher('/publish_topic', 'std_msgs/String')

for i in range(100):
    easy_publisher.pub(talker, data)
    time.sleep(0.5)
# Receiver Test
# Copyright 2017, Digi International Inc.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from digi.xbee.devices import XBeeDevice
import rospy
from std_msgs.msg import String
import sys

# The code should read your USB port from the second input of the command
# If not, please change the line here
PORT = "/dev/ttyUSB0"
if len(sys.argv) > 1:
    PORT = sys.argv[1]

# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 9600

def log_msg(msg):
    heading = "Xbee Receiver: "
    print(heading+msg)

def main():
    print(" +-----------------------------------------+")
    print(" | XBee Python Library Receive Data Sample |")
    print(" +-----------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()
        log_msg("Device Open")
        
        def data_receive_callback(xbee_message):
            decode_msg = xbee_message.data.decode()
            log_msg("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                     decode_msg))
            pub_string.publish(decode_msg)

        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")
        input()

    finally:
        if device is not None and device.is_open():
            print("Closed")
            device.close()


if __name__ == '__main__':

    # prepare ROS node
    rospy.init_node('xbee_receive_bridge',anonymous=False)

    # prepare ROS publisher
    pub_string = rospy.Publisher("xbee_receive", String, queue_size=1)

    main()

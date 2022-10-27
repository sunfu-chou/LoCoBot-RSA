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
import time

# The code should read your USB port from the second input of the command
# If not, please change the line here
PORT = "/dev/ttyUSB0"
if len(sys.argv) > 1:
    PORT = sys.argv[1]

# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 9600

device = XBeeDevice(PORT, BAUD_RATE)

mode = "WAIT" # WAIT or ACTIVE
msg_latch = ""
xbee_id = "0"
mymsg_id = None

def log_msg(msg):
    heading = "Xbee Operation: "
    print(heading+msg)

def main():
    print(" +-----------------------------------------+")
    print(" | XBee Python Library Receive Data Sample |")
    print(" +-----------------------------------------+\n")

    # device = XBeeDevice(PORT, BAUD_RATE)

    global xbee_id
    global mode
    global msg_latch
    global mymsg_id

    try:
        device.open()
        log_msg("Device Open")

        def data_receive_callback(xbee_message):
            global xbee_id
            global mode
            global msg_latch
            global mymsg_id
            decode_msg = xbee_message.data.decode()

            log_msg("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                     decode_msg))

            if len(decode_msg) == 5:
                if int(decode_msg[-1]) > int(xbee_id):
                    return

            if decode_msg[4] == xbee_id:
                pub_string.publish(msg_latch[4:])
                return
            msg_latch = decode_msg
            msg_id = decode_msg[0:4]
            to_send = msg_id + xbee_id
            time.sleep(0.1)
            # send_callback(to_send)
            print ("to send: ", to_send)
            print ("dev: ", xbee_message.remote_device)
            device.send_data_async(xbee_message.remote_device, to_send)
            

        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")
        input()

    finally:
        if device is not None and device.is_open():
            print("Closed")
            device.close()

def onShutdown():
    rospy.loginfo("Shutdown.")

if __name__ == '__main__':

    # prepare ROS node
    rospy.init_node('xbee_base',anonymous=False)

    # prepare ROS publisher
    pub_string = rospy.Publisher("xbee_receive", String, queue_size=1)

    main()
    rospy.on_shutdown(onShutdown)
    rospy.spin()

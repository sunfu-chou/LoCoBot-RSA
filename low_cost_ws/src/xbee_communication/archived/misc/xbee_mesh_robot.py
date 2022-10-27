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
import random

# The code should read your USB port from the second input of the command
# If not, please change the line here
PORT = "/dev/ttyUSB0"
if len(sys.argv) > 1:
    PORT = sys.argv[1]

# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 9600

device = XBeeDevice(PORT, BAUD_RATE)

mode = "WAIT"

def log_msg(msg):
    heading = "Xbee Operation: "
    print(heading+msg)

def cb(msg):
    global mode
    print ("CB")
    if mode == "SEND":
        mode = "WAIT"
        print("CB: SEND")
        decode = msg.data.decode()
        device.send_data(msg.remote_device, decode)
        

def send_callback(msg):
    global mode

    if mode == "SEND":
        return

    print("Sending broadcast data: %s..." % msg)

    msg_id = random.randint(1000, 9999)
    send_data = str(msg_id)+msg.data

    mode = "SEND"
    device.send_data_broadcast(send_data)

    print("Success")

    #wait_for_anchor()

def onShutdown():
    rospy.loginfo("Shutdown.")

if __name__ == '__main__':

    # prepare ROS node
    rospy.init_node('xbee_robot',anonymous=False)

    device.open()
    log_msg("Device Open")
    device.add_data_received_callback(cb)

    # prepare ROS subscriber
    sub_string = rospy.Subscriber("xbee_send", String, send_callback, queue_size=1)

    rospy.on_shutdown(onShutdown)
    rospy.spin()

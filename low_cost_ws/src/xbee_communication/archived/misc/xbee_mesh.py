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
from std_msgs.msg import String
import sys
import time
import random

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
xbee_id = "7"
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

    print("This is Xbee: ",xbee_id)

    try:
        device.open()
        log_msg("Device Open")
        device.set_sync_ops_timeout(0.3)

        def data_receive_callback(xbee_message):
            global xbee_id
            global mode
            global msg_latch
            global mymsg_id

            decode_msg = xbee_message.data.decode()
            log_msg("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                     decode_msg))

            if mode == "WAIT":
                if len(decode_msg) == 5:
                    if int(decode_msg[-1]) > int(xbee_id):
                        return
                if decode_msg[4] == xbee_id:
                    mode = "ACTIVE"
                    mymsg_id = msg_latch[0:4]
                    time.sleep(0.1)
                    send_callback(msg_latch)
                    return
                msg_latch = decode_msg
                msg_id = decode_msg[0:4]
                to_send = msg_id + xbee_id
                # time.sleep(0.1)
                # send_callback(to_send)
                device.send_data_async(xbee_message.remote_device, to_send)
            elif mode == "ACTIVE":
                get_id = int(decode_msg[4])
                my_id = int(xbee_id)

                itsmsg_id = int(decode_msg[0:4])
                ans = int(mymsg_id)

                print("My ID: ",my_id)
                print("Get ID: ",get_id)
                print("Its msgid: ",itsmsg_id)
                print("Ans: ",ans)
                if get_id < my_id and itsmsg_id == ans:
                    # send_callback(decode_msg)
                    device.send_data_async(xbee_message.remote_device, decode_msg)
                    mode = "WAIT"



        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")
        input()

    finally:
        if device is not None and device.is_open():
            print("Closed")
            device.close()

def send_callback(msg):

    print("Sending broadcast data: %s..." % msg)

    try:
        device.send_data_broadcast(msg)
    except:
        print("Weird Timeout Error")

    print("Success")

if __name__ == '__main__':

    import socket
    xbee_id = socket.gethostname()[-1]
    main()

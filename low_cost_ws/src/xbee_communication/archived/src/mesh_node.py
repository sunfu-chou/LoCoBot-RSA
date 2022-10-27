#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice
import random
import time
import rospy
from std_msgs.msg import Bool
import socket

node_list = {}
node_list['anchor01'] = 1
node_list['anchor02'] = 2
node_list['anchor03'] = 3
node_list['anchor04'] = 4
node_list['anchor05'] = 5
node_list['anchor06'] = 6
node_list['anchor07'] = 1
# node_list['anchor08'] = 2
node_list['anchor09'] = 2
node_list['anchor10'] = 3
node_list['anchor11'] = 4
node_list['anchor12'] = 5
node_list['anchor13'] = 6
node_list['anchor15'] = 1
node_list['anchor16'] = 2
node_list['anchor17'] = 3
node_list['anchor18'] = 4
node_list['anchor19'] = 5
node_list['anchor20'] = 6
node_list['anchor21'] = 1
node_list['anchor22'] = 2
node_list['anchor23'] = 3
node_list['anchor24'] = 4


NODE_NUMBER = node_list.get(socket.gethostname())

PORT = "/dev/xBee"
BAUD_RATE = 9600
device = XBeeDevice(PORT, BAUD_RATE)

MAX_MEMORY_SEC = 100.0
packetDict = { }
activate = False

def setup():
    device.open()
    print("device opened")

    device.add_data_received_callback(xbee_cb)
    print("ready to recieve")

    rospy.init_node("Comm_node_activation_listener")

def xbee_cb(msg):
    global activate
    addr = msg.remote_device.get_64bit_addr()
    selfAddr = str(device.get_64bit_addr())[-4:]
    data = msg.data.decode("utf8")
    packetID = data[:4]
    sourceAddr = data[4:8]
    message = data[8:]
    #print("Received data from %s" % addr)
    #print("Packet uid: %s" % packetID)
    #print("Source addr: %s" % sourceAddr)
    #print("message = %s" % message)

    if message[0] == "8":
        if int(message[1]) >= NODE_NUMBER:
            activate = True
    
    if packetID in packetDict:
        print("packet exist, ignore")
    else:
        packetDict[packetID] = [time.time(), message]
        currTime = time.time()
        key_copy = tuple(packetDict.keys())
        for pack in key_copy:
            if currTime - packetDict[pack][0] > MAX_MEMORY_SEC:
                packetDict.pop(pack)
        device.send_data_broadcast(data)

def cb_timer(self):
    msg = Bool()
    msg.data = activate
    pub_bool.publish(msg)


if __name__ == "__main__":
    setup()
    pub_bool = rospy.Publisher("activation", Bool, queue_size=1)
    timer = rospy.Timer(rospy.Duration(1), cb_timer)
    try:    
        rospy.spin()
    except KeyboardInterrupt:
        if device is not None and device.is_open():
            print("closing device on interrupt")
            device.close()

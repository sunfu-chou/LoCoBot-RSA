#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice
import random
import time
import rospy
from std_msgs.msg import Bool
import socket

node_list = {}
node_list['anchor01'] = 11
node_list['anchor02'] = 12
node_list['anchor03'] = 13
node_list['anchor04'] = 14
node_list['anchor05'] = 15
node_list['anchor06'] = 16
node_list['anchor07'] = 21
node_list['anchor08'] = 22
node_list['anchor09'] = 23
node_list['anchor10'] = 24
node_list['anchor11'] = 25
node_list['anchor13'] = 26
node_list['anchor15'] = 11
node_list['anchor16'] = 12
node_list['anchor17'] = 13
node_list['anchor18'] = 14
node_list['anchor19'] = 15
node_list['anchor20'] = 16
node_list['anchor21'] = 21
node_list['anchor22'] = 22
node_list['anchor23'] = 23
node_list['anchor24'] = 24


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
        if message[1] == str(NODE_NUMBER)[0]:
            if int(message[2]) >= NODE_NUMBER % 10:
                activate = True
    
    if sourceAddr == selfAddr:
        print("packet by me, ignore")
        return
    elif packetID in packetDict:
        print("packet exist, ignore")
    else:
        packetDict[packetID] = [time.time(), message]
        currTime = time.time()
        for pack in packetDict.keys():
            if currTime - packetDict[pack][0] > MAX_MEMORY_SEC:
                packetDict.pop(pack)
        echo_data = str(int(packetID)+1) + selfAddr + message
        device.send_data_broadcast(echo_data)

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

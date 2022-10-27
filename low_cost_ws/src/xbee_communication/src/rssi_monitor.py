#!/usr/bin/env python3

#Publish: neighbour and their rssi
import queue
import rospy
#import time
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.packets.common import ATCommPacket, ATCommResponsePacket
from digi.xbee.models.mode import OperatingMode
from xbee_communication.msg import NeighbourNode, NeighbourArray
from std_msgs.msg import String

class RSSI_Monitor(object):
    def __init__(self):
        self.PORT = rospy.get_param("~port")
        self.TO = rospy.get_param("~timeout")
        self.VEH = rospy.get_param("~veh")
        self.BAUD_RATE = 115200
        self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
        self.pub_rssi = rospy.Publisher("rssi_neighbour", NeighbourArray, queue_size=5)
        self.rate = rospy.Rate(1/self.TO)
        self.nodes = queue.Queue(maxsize=0)
        self.neighbours = {}

        self.device.open()
        self.device.add_packet_received_callback(self.packet_callback)

    def packet_callback(self, response):
        #decode
        rospy.loginfo("packet_cb")
        node = NeighbourNode()
        frame = response.get_frame_spec_data()
        node.addr_low = bytes(frame[11:15]).hex()
        node.node_id = frame[15:-10].decode()
        node.query_id = response.frame_id
        node.last_reported = rospy.Time.now()
        node.rssi = frame[-1]

        self.nodes.put(node)

    def add_node(self):
        node = self.nodes.get()

        #Find frame from the dicts
        #if found, update
        if node.addr_low in self.neighbours:
            self.neighbours[node.addr_low].query_id = node.query_id
            self.neighbours[node.addr_low].last_reported = node.last_reported
            self.neighbours[node.addr_low].rssi = node.rssi

        #else add new
        else:
            self.neighbours[node.addr_low] = node
        
        rospy.loginfo("%s,%s,%i,%f,%i",node.addr_low, node.node_id, node.query_id, node.last_reported.to_sec(), node.rssi)

    def publish(self, curr_frame_id):
        rospy.loginfo("Publishing")
        msg = NeighbourArray()
        msg.query_id = curr_frame_id
        msg.length = len(self.neighbours)
        for node in self.neighbours.values():
            msg.vec.append(node)

        self.pub_rssi.publish(msg)

    def on_shutdown(self):
        rospy.loginfo("Node Shutdown")
        if self.device is not None and self.device.is_open():
            self.device.close()

if __name__ == "__main__":
    rospy.init_node("rssi_monitor_node")

    monitor = RSSI_Monitor()
    try:
        while True:
            try:
                curr_frame_id = monitor.device._get_next_frame_id()
                request = ATCommPacket(curr_frame_id, "FN")
                print("Send Request", curr_frame_id)
                monitor.device.send_packet(request)
                
                monitor.rate.sleep()
                while not monitor.nodes.empty():
                    monitor.add_node()
                monitor.publish(curr_frame_id)
            except (KeyboardInterrupt, SystemExit):
                raise

    finally:
        monitor.on_shutdown()

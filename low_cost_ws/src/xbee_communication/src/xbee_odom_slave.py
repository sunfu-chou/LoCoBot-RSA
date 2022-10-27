#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from xbee_communication.srv import xbee
from xbee_communication.msg import Robot_PoseStamped, xbee_trans_speed
import struct
import roslib
import pickle
import time
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import *
from digi.xbee.models.address import *
from digi.xbee.packets.base import DictKeys
from digi.xbee.packets.common import ATCommPacket, ATCommResponsePacket
from digi.xbee.models.mode import OperatingMode

from datetime import datetime


class XBee(object):
	def __init__(self):
		self.PORT = rospy.get_param("~port")
		self.BAUD_RATE = 115200
		self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
		self.device.open(force_settings=True)

		self.sourceAddr = str(self.device.get_64bit_addr())
		# self.service = rospy.Service('xbee', xbee, self.handle_ros_service)
		self.data_odom, self.data_twist = [], []
		self.check, self.get_register, self.data_bytes, self.last_remote = 0, bytearray(), 0, None
		self.I_am_base_station = rospy.get_param("~I_am_base_station")
		self.get_ACK, self.get_array_checksum, self.PointsEnd = None, None, 0
		self.address_to_robot = {	rospy.get_param("/xbee_address/base_station"):"base_station", \
									rospy.get_param("/xbee_address/boat1"):"boat1", \
									rospy.get_param("/xbee_address/boat2"):"boat2", \
									rospy.get_param("/xbee_address/boat3"):"boat3", \
									rospy.get_param("/xbee_address/boat4"):"boat4"  }
		self.robot_to_address = {v: k for k, v in self.address_to_robot.items()}
		self.all_Publisher  = dict()
		for robot in ["boat1", "boat2", "boat3", "boat4"]:
			base_pub = dict()
			for ask in ["AskTwist", "AskPose"]: base_pub[ask] = None
			self.all_Publisher[robot] = base_pub

		# for MOOS, all robot have to know where others are
		self.all_Publisher["boat1"]["AskTwist"] = rospy.Publisher("boat1/robot_twist", Twist, queue_size=1)
		self.all_Publisher["boat2"]["AskTwist"] = rospy.Publisher("boat2/robot_twist", Twist, queue_size=1)
		self.all_Publisher["boat3"]["AskTwist"] = rospy.Publisher("boat3/robot_twist", Twist, queue_size=1)
		self.all_Publisher["boat4"]["AskTwist"] = rospy.Publisher("boat4/robot_twist", Twist, queue_size=1)
		self.sub_robot_odom = rospy.Subscriber("/wamv3/localization_gps_imu/odometry",Odometry,self.cb_robot_odom,queue_size = 1)
		self.pub_Robot_cmd = rospy.Publisher("/wamv3/cmd_vel", Twist, queue_size=1)

		self.use_robot = []
		if rospy.get_param("~use_boat1") : self.use_robot.append("boat1")
		if rospy.get_param("~use_boat2") : self.use_robot.append("boat2")
		if rospy.get_param("~use_boat3") : self.use_robot.append("boat3")
		if rospy.get_param("~use_boat4") : self.use_robot.append("boat4")

		self.device.add_data_received_callback(self.xbee_callback_and_decode)
		self.sending_break = False
		# self.device.add_packet_received_callback(self.packet_received_callback)
		print("xbee node initialized, I am ", self.sourceAddr[8:])
		print('use_robot = ',self.use_robot)





	def cb_robot_odom(self,msg):
		self.data_odom = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,\
		msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, \
							msg.pose.pose.orientation.w,msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z,\
							msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]\
							,dtype=np.float16)

		

	def np_array_to_twist(self,array):
		msg = Twist()
		msg.linear.x = array[0]
		msg.linear.y = array[1]
		msg.linear.z = array[2]
		msg.angular.x = array[3]
		msg.angular.y = array[4]
		msg.angular.z = array[5]
		return msg




	def ask_robot_data(self,robot,ask):
                print('------ ', datetime.now().strftime("%H:%M:%S "),robot,' ',ask,' ------')
                self.get_ACK, self.get_array_checksum = None, False
                speed_msg = xbee_trans_speed()
                if self.xbee_encode_and_send(self.robot_to_address[robot],ask,data_type=b'\x00'):
                        time0 = time.time()
                        while self.get_ACK == None :
                            time.sleep(0.01)
                            if ((time.time() - time0) > 1) :
                                print('wait ACK TimeOut',self.get_ACK)
                                self.clear()
                                break
                        speed_msg.xbee_trans_speed = 256/(time.time()-time0)
                        if (self.get_ACK == True) and (ask=="AskTwist"):
                            time1 = time.time()
                            last_len = 0
                            while not self.get_array_checksum:
                                time.sleep(0.01)
                                if (time.time() - time1) > 1 :
                                    if len(self.get_register) == last_len:
                                        print('wait following TimeOut')
                                        self.clear()
                                        break
                                    last_len = len(self.get_register)
                                    time1 = time.time()
                            if self.get_array_checksum :
                                self.clear()
                                return True
                        speed_msg.address = robot
                        self.xbee_trans_speed_pub.publish(speed_msg)
                return False


	def clear(self):
		self.check, self.get_register, self.data_bytes, self.get_ACK, self.get_array_checksum = 0, bytearray(), 0, None, None

	# def handle_ros_service(self,req): # deal with the client (xbee_client.py)
	# 	if req.address in ["boat1", "boat2", "boat3", "boat4"]: ADDRESS_L = self.robot_to_address[req.address]
	# 	else : return False

	# 	if req.message == "Move":
	# 		goal_array = np.array([req.x,req.y,req.z],dtype=np.float16)
	# 		return self.xbee_encode_and_send(ADDRESS_L, goal_array, data_type=b'\x03') #send goal array
	# 	elif ((req.message == "AskPose") or (req.message == "AskPoints")):
	# 		return self.ask_robot_data(req.address,req.message) # robot = req.address, aks = req.message
	# 	else:
	# 		return self.xbee_encode_and_send(ADDRESS_L, req.message, data_type=b'\x00') #send string msg


	def xbee_callback_and_decode(self, xbee_message):
		ADDRESS = str(xbee_message.remote_device.get_64bit_addr()) # msg from who
		if not self.last_remote == ADDRESS : self.clear()
		self.last_remote = ADDRESS

		if not xbee_message.data[0:1] == b'\xAB' : # Header wrong
			self.clear()
			print('get xbee_message with wrong Header')
			return

		if not ((xbee_message.data[1:2] == b'\x00') or (xbee_message.data[1:2] == b'\x01') or (xbee_message.data[1:2] == b'\x02') or (xbee_message.data[1:2] == b'\x03')):
			self.clear()
			print('get xbee_message with wrong type')
			return

		self.get_register.extend(xbee_message.data[6:])
		self.data_bytes = int.from_bytes(xbee_message.data[2:6], byteorder="big",signed=False)


		if (self.data_bytes == len(self.get_register) -1): # the last one of data pkgs
                    #print(xbee_message.data[1:2])
                    if xbee_message.data[1:2] == b'\x00': # type: string msg
                        self.sending_break = True
                        get_msg = pickle.loads(self.get_register[:-1])
                        self.clear()
                        print('get string msg= ',get_msg)
                        if get_msg == "AskTwist":
                            self.get_new_ask = True
                            print(" AskTwist from ", ADDRESS[8:])
                            if self.data_twist == []:
                                self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
                            else :
                                self.xbee_encode_and_send(ADDRESS[8:],'OK', data_type=b'\x00') #send string msg
                                self.xbee_encode_and_send(ADDRESS[8:], self.data_twist, data_type=b'\x01') #send twist
                                self.data_twist = []
                        elif get_msg == "AskPose":
                            self.get_new_ask = True
                            print(" AskPose from ", ADDRESS[8:])
                            if self.data_odom == []:
                                self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
                            else :
                                self.xbee_encode_and_send(ADDRESS[8:],'OK',data_type=b'\x00')
                                self.xbee_encode_and_send(ADDRESS[8:],self.data_odom,data_type=b'\x02')
                                self.data_odom=[]
                                self.ask_robot_data("base_station","AskTwist")
                        elif get_msg == "OK": self.get_ACK = True
                        elif get_msg == "Not OK": self.get_ACK = "Not OK"
                    elif xbee_message.data[1:2] == b'\x01': # type: points
                        get_points = pickle.loads(self.get_register[:-1])
                        self.clear()
                        pub_msg = self.np_array_to_twist(get_points)
                        print(pub_msg)
                        self.pub_Robot_cmd.publish(pub_msg)
                        self.get_array_checksum = True

                    elif xbee_message.data[1:2] == b'\x02': # type: pose
                        get_odom = pickle.loads(self.get_register[:-1])
                        self.clear()
                        pub_msg = self.np_array_to_Odometry(get_odom)
                        (self.all_Publisher[self.address_to_robot[ADDRESS[8:]]]["AskPose"]).publish(pub_msg)
                        print(get_odom)
                        self.get_array_checksum = True





	def xbee_encode_and_send(self, ADDRESS_L, data_via_xbee, data_type):
		data = data_via_xbee
		ADDRESS_H = '0013A200'
		ADDRESS = ADDRESS_H + ADDRESS_L

		# send data
		byte_arr = pickle.dumps( data )
		length, index, check= int(len(byte_arr)), 0, 0

		for index in range(0,length,250) :
			pack = bytearray(b'\xAB') #Header
			pack.extend(bytearray(data_type)) #Type
			pack.extend( length.to_bytes(4, byteorder='big') ) #bytes
			index_end = index+250 if index+250 < length else length
			pack.extend( byte_arr[index:(index_end)] ) #data

			if index_end == length : pack.extend(check.to_bytes(1, byteorder='big')) # checksum
			else: check = 0xff & (check + pack[-1])
			if data_type == b'\x02' : self.device.send_data_broadcast(pack) # all robot have to know where others are
			else:
				try:
					self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
				except XBeeException:
					print('digi.xbee.exception.XBeeException: XBee devices serial port closed')
					self.clear()
					return False
				except TransmitException:
					print('digi.xbee.exception.TransmitException: There was a problem with a transmitted packet response')
					self.clear()
					return False
				except TimeoutException:
					print('digi.xbee.exception.TimeoutException: Response not received in the configured timeout.')
					self.clear()
					return False
				except :
					print('send data_via_xbee fail')
					self.clear()
					return False

		return True


if __name__ == "__main__":
	rospy.init_node("xbee_node")
	xbee_node = XBee()
	rospy.spin()

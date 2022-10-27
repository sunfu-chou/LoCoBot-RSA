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
		self.count = 1

		self.sourceAddr = str(self.device.get_64bit_addr())
		# self.service = rospy.Service('xbee', xbee, self.handle_ros_service)
		self.data_odom, self.data_twist = [], []
		self.check, self.get_register, self.data_bytes, self.last_remote = 0, bytearray(), 0, None
		self.I_am_base_station = rospy.get_param("~I_am_base_station")
		self.timer = rospy.Timer(rospy.Duration(0.7), self.auto_ask_timer)
		self.get_ACK, self.get_array_checksum, self.PointsEnd = None, None, 0
		self.address_to_robot = {	rospy.get_param("/xbee_address/boat1"):"boat1", \
									rospy.get_param("/xbee_address/boat2"):"boat2", \
									rospy.get_param("/xbee_address/boat3"):"boat3", \
									rospy.get_param("/xbee_address/boat4"):"boat4", \
									rospy.get_param("/xbee_address/boat5"):"boat5", \
									rospy.get_param("/xbee_address/boat6"):"boat6", \
									rospy.get_param("/xbee_address/boat7"):"boat7"
									  }
		self.robot_to_address = {v: k for k, v in self.address_to_robot.items()}
		self.all_Publisher  = dict()
		for robot in ["boat1", "boat2", "boat3", "boat4", "boat5", "boat6", "boat7"]:
			base_pub = dict()
			for ask in ["AskPose", "AskTwist"]: base_pub[ask] = None
			self.all_Publisher[robot] = base_pub

		# for MOOS, all robot have to know where others are
		self.all_Publisher["boat1"]["AskPose"] = rospy.Publisher("boat1/robot_pose", Odometry, queue_size=1)
		self.all_Publisher["boat2"]["AskPose"] = rospy.Publisher("boat2/robot_pose", Odometry, queue_size=1)
		self.all_Publisher["boat3"]["AskPose"] = rospy.Publisher("boat3/robot_pose", Odometry, queue_size=1)
		self.all_Publisher["boat4"]["AskPose"] = rospy.Publisher("boat4/robot_pose", Odometry, queue_size=1)
		self.all_Publisher["boat5"]["AskPose"] = rospy.Publisher("boat5/robot_pose", Odometry, queue_size=1)
		self.all_Publisher["boat6"]["AskPose"] = rospy.Publisher("boat6/robot_pose", Odometry, queue_size=1)
		self.all_Publisher["boat7"]["AskPose"] = rospy.Publisher("boat7/robot_pose", Odometry, queue_size=1)




		self.sub_Robot_cmd_1 = rospy.Subscriber("wamv/cmd_vel", Twist, self.cb_Robot_cmd_1, queue_size=1)
		self.sub_Robot_smd_2 = rospy.Subscriber("wamv1/cmd_vel",Twist,self.cb_Robot_cmd_2,queue_size=1)
		self.sub_Robot_smd_3 = rospy.Subscriber("wamv2/cmd_vel",Twist,self.cb_Robot_cmd_3,queue_size=1)
		self.sub_Robot_smd_4 = rospy.Subscriber("wamv3/cmd_vel",Twist,self.cb_Robot_cmd_4,queue_size=1)
		self.sub_Robot_smd_5 = rospy.Subscriber("wamv4/cmd_vel",Twist,self.cb_Robot_cmd_5,queue_size=1)
		self.sub_Robot_smd_6 = rospy.Subscriber("wamv5/cmd_vel",Twist,self.cb_Robot_cmd_6,queue_size=1)
		self.sub_Robot_smd_7 = rospy.Subscriber("wamv6/cmd_vel",Twist,self.cb_Robot_cmd_7,queue_size=1)
		self.xbee_trans_speed_pub = rospy.Publisher("xbee_trans_speed", xbee_trans_speed, queue_size=1)


		self.use_robot = []
		if rospy.get_param("~use_boat1") : self.use_robot.append("boat1")
		if rospy.get_param("~use_boat2") : self.use_robot.append("boat2")
		if rospy.get_param("~use_boat3") : self.use_robot.append("boat3")
		if rospy.get_param("~use_boat4") : self.use_robot.append("boat4")
		if rospy.get_param("~use_boat5") : self.use_robot.append("boat5")
		if rospy.get_param("~use_boat6") : self.use_robot.append("boat6")
		if rospy.get_param("~use_boat7") : self.use_robot.append("boat7")

		self.device.add_data_received_callback(self.xbee_callback_and_decode)
		self.sending_break = False
		# self.device.add_packet_received_callback(self.packet_received_callback)
		print("xbee node initialized, I am ", self.sourceAddr[8:])
		print('use_robot = ',self.use_robot)






	# for base_station
	def cb_Robot_cmd_1(self, msg):

		self.data_twist_1 = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)

	def cb_Robot_cmd_2(self, msg):

		self.data_twist_2 = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)
	
	def cb_Robot_cmd_3(self, msg):

		self.data_twist_3 = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)
	def cb_Robot_cmd_4(self, msg):

		self.data_twist_4 = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)

	def cb_Robot_cmd_5(self, msg):

		self.data_twist_5 = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)

	def cb_Robot_cmd_6(self, msg):

		self.data_twist_6 = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)

	def cb_Robot_cmd_7(self, msg):

		self.data_twist_7 = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)




		


	def np_array_to_Odometry(self,array):
		msg = Odometry()
		msg.header.frame_id = "odom"
		msg.pose.pose.position.x = array[0]
		msg.pose.pose.position.y = array[1]
		msg.pose.pose.position.z = array[2]
		msg.pose.pose.orientation.x = array[3]
		msg.pose.pose.orientation.y = array[4]
		msg.pose.pose.orientation.z = array[5]
		msg.pose.pose.orientation.w = array[6]
		msg.twist.twist.linear.x = array[7]
		msg.twist.twist.linear.y = array[8]
		msg.twist.twist.linear.z = array[9]
		msg.twist.twist.angular.x = array[10]
		msg.twist.twist.angular.y = array[11]
		msg.twist.twist.angular.z = array[12]
		return msg


	def auto_ask_timer(self,event):
		
		if self.count == 1:
			self.ask_robot_data("boat1","AskPose")
		elif self.count == 2:
			self.ask_robot_data("boat2","AskPose")
		elif self.count == 3:
			self.ask_robot_data("boat3","AskPose")
		elif self.count == 4:
			self.ask_robot_data("boat4","AskPose")
		elif self.count == 5:
			self.ask_robot_data("boat5","AskPose")
		elif self.count == 6:
			self.ask_robot_data("boat6","AskPose")
		elif self.count == 7:
			self.ask_robot_data("boat7","AskPose")
			self.count = 0

		self.count = self.count +1



	def ask_robot_data(self,robot,ask):
		print('------ ', datetime.now().strftime("%H:%M:%S "),robot,' ',ask,' ------')
		self.get_ACK, self.get_array_checksum = None, False
		speed_msg = xbee_trans_speed()
		print(self.robot_to_address[robot])
		if self.xbee_encode_and_send(self.robot_to_address[robot], ask, data_type=b'\x00') :
			time0 = time.time()
			while self.get_ACK == None :
				time.sleep(0.01)
				if ((time.time() - time0) > 0.5) :
					print('wait ACK TimeOut',self.get_ACK)
					self.clear()
					break
			speed_msg.xbee_trans_speed = 256/(time.time()-time0)

			if (self.get_ACK == True) and (ask=="AskPose"):
				time1 = time.time()
				last_len = 0
				while not self.get_array_checksum :
					time.sleep(0.01)
					if (time.time() - time1) > 0.5 :
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
			if xbee_message.data[1:2] == b'\x00': # type: string msg
				self.sending_break = True

				get_msg = pickle.loads(self.get_register[:-1])
				self.clear()
				print('get string msg= ',get_msg)
				if get_msg == "AskTwist":
					self.get_new_ask = True
					print(" AskTwist from ", ADDRESS[8:])
					if self.data_twist_1 == []:
						self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
					else :
						self.xbee_encode_and_send(ADDRESS[8:],'OK', data_type=b'\x00') #send string msg
						if(ADDRESS[8:]==self.robot_to_address["boat1"]):
							self.xbee_encode_and_send(ADDRESS[8:], self.data_twist_1, data_type=b'\x01') #send twist
						elif(ADDRESS[8:]==self.robot_to_address["boat2"]):
							self.xbee_encode_and_send(ADDRESS[8:], self.data_twist_2, data_type=b'\x01')
						elif(ADDRESS[8:]==self.robot_to_address["boat3"]):
							self.xbee_encode_and_send(ADDRESS[8:], self.data_twist_3, data_type=b'\x01')
						elif(ADDRESS[8:]==self.robot_to_address["boat4"]):
							self.xbee_encode_and_send(ADDRESS[8:], self.data_twist_4, data_type=b'\x01')
						elif(ADDRESS[8:]==self.robot_to_address["boat5"]):
							self.xbee_encode_and_send(ADDRESS[8:], self.data_twist_5, data_type=b'\x01')
						elif(ADDRESS[8:]==self.robot_to_address["boat6"]):
							self.xbee_encode_and_send(ADDRESS[8:], self.data_twist_6, data_type=b'\x01')
						elif(ADDRESS[8:]==self.robot_to_address["boat7"]):
							print("send to boat7")
							print(self.data_twist_7)
							self.xbee_encode_and_send(ADDRESS[8:], self.data_twist_7, data_type=b'\x01') 
						self.data_twist = []
				elif get_msg == "AskPose":
					self.get_new_ask = True
					print(" AskPose from ", ADDRESS[8:])
					if self.data_odom == []:
						self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
					else :
						self.xbee_encode_and_send(ADDRESS[8:],'OK', data_type=b'\x00') #send string msg
						self.xbee_encode_and_send(ADDRESS[8:], self.data_odom, data_type=b'\x02') #send pose
						self.xbee_encode_and_send(self.robot_to_address[robot],"AskTwist" , data_type=b'\x00')
						self.data_odom = []

				elif get_msg == "OK": self.get_ACK = True
				elif get_msg == "Not OK": self.get_ACK = "Not OK"

			elif xbee_message.data[1:2] == b'\x01': # type: points
				get_points = pickle.loads(self.get_register[:-1])
				self.clear()
				pub_msg = self.np_array_to_twist(get_points)
				(self.all_Publisher[self.address_to_robot[ADDRESS[8:]]]["AskTwist"]).publish(pub_msg)
				# print(get_points)
				self.get_array_checksum = True
				self.PointsEnd = self.PointsEnd + len(get_points)


			elif xbee_message.data[1:2] == b'\x02': # type: pose
				get_odom = pickle.loads(self.get_register[:-1])
				self.clear()
				pub_msg = self.np_array_to_Odometry(get_odom)
				(self.all_Publisher[self.address_to_robot[ADDRESS[8:]]]["AskPose"]).publish(pub_msg)
				print("publish")
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

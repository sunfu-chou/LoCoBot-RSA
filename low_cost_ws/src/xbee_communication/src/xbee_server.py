#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
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
		self.service = rospy.Service('xbee', xbee, self.handle_ros_service)
		self.data_points, self.data_pose = [], []
		self.check, self.get_register, self.data_bytes, self.last_remote = 0, bytearray(), 0, None
		self.I_am_base_station = rospy.get_param("~I_am_base_station")
		self.timer = rospy.Timer(rospy.Duration(1), self.auto_ask_timer)
		self.get_ACK, self.get_array_checksum, self.PointsEnd = None, None, 0
		self.address_to_robot = {	rospy.get_param("/xbee_address/husky1"):"husky1", \
									rospy.get_param("/xbee_address/husky2"):"husky2", \
									rospy.get_param("/xbee_address/jackal1"):"jackal1", \
									rospy.get_param("/xbee_address/jackal2"):"jackal2"  }
		self.robot_to_address = {v: k for k, v in self.address_to_robot.items()}
		self.all_Publisher  = dict()
		for robot in ["husky1", "husky2", "jackal1", "jackal2"]:
			base_pub = dict()
			for ask in ["AskPoints", "AskPose"]: base_pub[ask] = None
			self.all_Publisher[robot] = base_pub

		# for MOOS, all robot have to know where others are
		self.all_Publisher["husky1"]["AskPose"] = rospy.Publisher("husky1/robot_pose", PoseStamped, queue_size=1)
		self.all_Publisher["husky2"]["AskPose"] = rospy.Publisher("husky2/robot_pose", PoseStamped, queue_size=1)
		self.all_Publisher["jackal1"]["AskPose"] = rospy.Publisher("jackal1/robot_pose", PoseStamped, queue_size=1)
		self.all_Publisher["jackal2"]["AskPose"] = rospy.Publisher("jackal2/robot_pose", PoseStamped, queue_size=1)

		if self.I_am_base_station :
			self.sub_Robot_PoseStamped = rospy.Subscriber("/car_cmd", Robot_PoseStamped, self.cb_Robot_PoseStamped, queue_size=1)
			self.car_cmd_FIFO = []
			self.all_Publisher["husky1"]["AskPoints"] = rospy.Publisher("husky1/robot_points", PointCloud2, queue_size=1)
			self.all_Publisher["husky2"]["AskPoints"] = rospy.Publisher("husky2/robot_points", PointCloud2, queue_size=1)
			self.all_Publisher["jackal1"]["AskPoints"] = rospy.Publisher("jackal1/robot_points", PointCloud2, queue_size=1)
			self.all_Publisher["jackal2"]["AskPoints"] = rospy.Publisher("jackal2/robot_points", PointCloud2, queue_size=1)
			self.xbee_trans_speed_pub = rospy.Publisher("xbee_trans_speed", xbee_trans_speed, queue_size=1)

		if not self.I_am_base_station : # I am a robot
			self.sub_points = rospy.Subscriber("map_part", PointCloud2, self.cb_points, queue_size=1)
			self.sub_pose = rospy.Subscriber("slam_pose", PoseStamped, self.cb_pose, queue_size=1)
			self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

		self.use_robot = []
		if rospy.get_param("~use_husky1") : self.use_robot.append("husky1")
		if rospy.get_param("~use_husky2") : self.use_robot.append("husky2")
		if rospy.get_param("~use_jackal1") : self.use_robot.append("jackal1")
		if rospy.get_param("~use_jackal2") : self.use_robot.append("jackal2")

		self.device.add_data_received_callback(self.xbee_callback_and_decode)
		self.sending_break = False
		# self.device.add_packet_received_callback(self.packet_received_callback)
		print("xbee node initialized, I am ", self.sourceAddr[8:])
		print('use_robot = ',self.use_robot)








	# for robot
	def cb_points(self, msg): # PointCloud2 to numpy
		cloud_points = []
		for p in point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True): cloud_points.append(p)
		self.data_points = np.array(cloud_points, dtype=np.float16)

	def cb_pose(self, msg):
		self.data_pose = np.array( [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, \
								msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w] ,dtype=np.float16)

	def moving_to_goal(self,x,y,z):
		goal_msg = PoseStamped()
		goal_msg.header.frame_id = "map"
		goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z = x, y, z
		self.pub_goal.publish(goal_msg)
		print("I am moving to ",x,y,z,' ~~~')



	# for base_station
	def cb_Robot_PoseStamped(self, msg):
		robot = msg.robot_name
		x = msg.posestamped.pose.position.x
		y = msg.posestamped.pose.position.y
		z = msg.posestamped.pose.position.z
		self.car_cmd_FIFO.append( (robot,[x,y,z]) )
		return

	def np_array_to_PointCloud2(self, points):
		msg = PointCloud2()
		buf = []
		msg.header.frame_id = "map"
		if len(points.shape) == 3:
			msg.height = points.shape[1]
			msg.width = points.shape[0]
		else:
			N = len(points)
			xyzrgb = np.array(np.hstack([points]), dtype=np.float32)
			msg.height = 1
			msg.width = N
		msg.fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1)
		]
		msg.is_bigendian = False
		msg.point_step = 12
		msg.row_step = msg.point_step * N
		msg.is_dense = True
		msg.data = xyzrgb.tostring()
		return msg

	def np_array_to_PoseStamped(self,array):
		msg = PoseStamped()
		msg.header.frame_id = "map"
		msg.pose.position.x = array[0]
		msg.pose.position.y = array[1]
		msg.pose.position.z = array[2]
		msg.pose.orientation.x = array[3]
		msg.pose.orientation.y = array[4]
		msg.pose.orientation.z = array[5]
		msg.pose.orientation.w = array[6]
		return msg

	def auto_ask_timer(self,event):
		if not self.I_am_base_station: return
		# ask robots to get data
		for ask in ["AskPose", "AskPose", "AskPose", "AskPose", "AskPose", "AskPoints"]:
			for robot in self.use_robot:
				self.ask_robot_data(robot,ask)

				# make robot move
				while not (len(self.car_cmd_FIFO) == 0) :
					tmp = self.car_cmd_FIFO.pop(0)
					goal_array = np.array([tmp[1][0],tmp[1][1],tmp[1][2]],dtype=np.float16)
					ADDRESS_L = self.robot_to_address[ tmp[0] ]
					if self.xbee_encode_and_send(ADDRESS_L, goal_array, data_type=b'\x03'):
						print('sending moving xbee msg success',tmp[0],goal_array)
					else:
						print('sending moving xbee msg fail',tmp[0],goal_array)


	def ask_robot_data(self,robot,ask):
		print('------ ', datetime.now().strftime("%H:%M:%S "),robot,' ',ask,' ------')
		self.get_ACK, self.get_array_checksum = None, False
		speed_msg = xbee_trans_speed()

		if self.xbee_encode_and_send(self.robot_to_address[robot], ask, data_type=b'\x00') :
			time0 = time.time()
			while self.get_ACK == None :
				time.sleep(0.01)
				if ((time.time() - time0) > 1) :
					print('wait ACK TimeOut',self.get_ACK)
					self.clear()
					break
			speed_msg.xbee_trans_speed = 256/(time.time()-time0)

			if (self.get_ACK == True) and (ask=="AskPose"):
				time1 = time.time()
				last_len = 0
				while not self.get_array_checksum :
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

			if (self.get_ACK == True) and (ask=="AskPoints"):
				time2 = time.time()
				last_point_num = 0
				while not (self.PointsEnd == "Done"):
					time.sleep(0.01)
					if (time.time() - time2) > 1 :
						if self.PointsEnd == last_point_num:
							print('wait following points TimeOut')
							self.clear()
							break
						print('Keep receiving ',self.PointsEnd,' points have get')
						last_point_num = self.PointsEnd
						time2 = time.time()

				self.clear()
				self.PointsEnd = 0
				return True
			speed_msg.address = robot
			self.xbee_trans_speed_pub.publish(speed_msg)
		return False


	def clear(self):
		self.check, self.get_register, self.data_bytes, self.get_ACK, self.get_array_checksum = 0, bytearray(), 0, None, None

	def handle_ros_service(self,req): # deal with the client (xbee_client.py)
		if req.address in ["husky1", "husky2", "jackal1", "jackal2"]: ADDRESS_L = self.robot_to_address[req.address]
		else : return False

		if req.message == "Move":
			goal_array = np.array([req.x,req.y,req.z],dtype=np.float16)
			return self.xbee_encode_and_send(ADDRESS_L, goal_array, data_type=b'\x03') #send goal array
		elif ((req.message == "AskPose") or (req.message == "AskPoints")):
			return self.ask_robot_data(req.address,req.message) # robot = req.address, aks = req.message
		else:
			return self.xbee_encode_and_send(ADDRESS_L, req.message, data_type=b'\x00') #send string msg


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
				if get_msg == "AskPoints":
					self.get_new_ask = True
					print(" AskPoints from ", ADDRESS[8:])
					if self.data_points == []:
						self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
					else :
						self.sending_break = False
						self.xbee_encode_and_send(ADDRESS[8:],'OK', data_type=b'\x00') #send string msg
						for i in range(0,len(self.data_points),16):
							if self.sending_break : break
							print('sending points: ',i,'/',len(self.data_points))
							self.xbee_encode_and_send(ADDRESS[8:],self.data_points[i:i+16], data_type=b'\x01') #send points
						self.xbee_encode_and_send(ADDRESS[8:],'Points End', data_type=b'\x00') #send string msg
						self.data_points = []
				elif get_msg == "AskPose":
					self.get_new_ask = True
					print(" AskPose from ", ADDRESS[8:])
					if self.data_pose == []:
						self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
					else :
						self.xbee_encode_and_send(ADDRESS[8:],'OK', data_type=b'\x00') #send string msg
						self.xbee_encode_and_send(ADDRESS[8:], self.data_pose, data_type=b'\x02') #send pose
						self.data_pose = []

				elif get_msg == "OK": self.get_ACK = True
				elif get_msg == "Not OK": self.get_ACK = "Not OK"
				elif get_msg == 'Points End': self.PointsEnd = "Done"

			elif xbee_message.data[1:2] == b'\x01': # type: points
				get_points = pickle.loads(self.get_register[:-1])
				self.clear()
				pub_msg = self.np_array_to_PointCloud2(get_points)
				(self.all_Publisher[self.address_to_robot[ADDRESS[8:]]]["AskPoints"]).publish(pub_msg)
				# print(get_points)
				self.get_array_checksum = True
				self.PointsEnd = self.PointsEnd + len(get_points)


			elif xbee_message.data[1:2] == b'\x02': # type: pose
				get_pose = pickle.loads(self.get_register[:-1])
				self.clear()
				pub_msg = self.np_array_to_PoseStamped(get_pose)
				(self.all_Publisher[self.address_to_robot[ADDRESS[8:]]]["AskPose"]).publish(pub_msg)
				print(get_pose)
				self.get_array_checksum = True


			elif xbee_message.data[1:2] == b'\x03': # type: goal
				get_goal = pickle.loads(self.get_register[:-1])
				self.clear()
				print('get_goal= ',get_goal)
				self.moving_to_goal(get_goal[0], get_goal[1], get_goal[2])




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

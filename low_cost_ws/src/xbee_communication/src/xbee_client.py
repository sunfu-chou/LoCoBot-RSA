#!/usr/bin/env python3

import rospy
from xbee_communication.srv import xbee
import random
import time
import sys


if __name__ == "__main__":
	if len(sys.argv) == 3:
		add = str(sys.argv[1])
		msg = str(sys.argv[2])
		x = float(0)
		y = float(0)
		z = float(0)
	elif len(sys.argv) == 6:
		add = str(sys.argv[1])
		msg = str(sys.argv[2])
		x = float(sys.argv[3])
		y = float(sys.argv[4])
		z = float(sys.argv[5])
	else:
		print('input with wrong sys.argv')
		sys.exit(1)

	rospy.wait_for_service('xbee')
	try:
		handle_xbee = rospy.ServiceProxy('xbee', xbee)
		result = handle_xbee(add, msg, x, y, z)
		print(result)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

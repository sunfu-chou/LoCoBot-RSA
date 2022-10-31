#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import *
from geometry_msgs.msg import Twist

class Base_controller:
  def __init__(self):
    self.node_name = rospy.get_name()
    rospy.loginfo("[%s] Initializing " %(self.node_name))

    self.joy_sub = rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size=1)
    self.cmd_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

    self.twist_msg = Twist()
    rate = rospy.Rate(10)
    self.control_flag = True # enable joystick control

    while not rospy.is_shutdown():
      if self.control_flag == True:
        self.cmd_pub.publish(self.twist_msg)
      rate.sleep()

  def JoyCallback(self, msg):
    if msg.buttons[7] == 1:
        if self.control_flag == False:
          print("joystick mode")
        self.control_flag = True
    if msg.buttons[6] == 1:
        if self.control_flag == True:
          print("auto mode")
        self.control_flag = False

    if self.control_flag == True:
        linear = msg.axes[1]
        angular = msg.axes[3]
        self.twist_msg.linear.x = linear*0.35
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
       
        self.twist_msg.angular.z = angular
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        #self.cmd_pub.publish(self.twist_msg)
        rospy.loginfo("(%f, %f)", linear,angular)

if __name__ == "__main__":
  rospy.init_node("joystick_control",anonymous=False)
  joystick = Base_controller()
  rospy.spin()
    

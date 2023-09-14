#!/usr/bin/env python
# coding: utf-8



from arg_robotics_tools import websocket_rosbridge as socket
from pynput import keyboard
import threading
import time
import os


class base_control():
    def __init__(self, ip):
        self.data = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        self.ip = ip
        #################YOUR CODE HERE#####################
        #you need create a ros_socket connection to ip and port 9090 
        #and create a publisher by your ros_socket connection to topic named 'cmd_vel_mux/input/teleop', and its type is 'geometry_msgs/Twist'
        
        ####################################################
        t = threading.Thread(target=self.pub)
        t.start()

    def pub(self):
        while True:
            #################YOUR CODE HERE#####################
            #you need to publish your data by your publisher with ros_socket connection
            
            #print(self.data)
            ####################################################
            time.sleep(0.1) #you can change the sleep time to control the publish rate

    def on_press(self, key):
        try:
            #####the limit of the robot#####
            #linear velocity: -2 ~ 2
            #angular velocity: -0.5 ~ 0.5
            ################################

            if key == keyboard.Key.up:
                print('up')
                #################YOUR CODE HERE#####################
                #you need to change the data when up arrow key is press
                
                ####################################################
            elif key == keyboard.Key.down:
                print('down')
                #################YOUR CODE HERE#####################
                #you need to change the data when down arrow key is press
                
                ####################################################
            elif key == keyboard.Key.left:
                print('left')
                #################YOUR CODE HERE#####################
                #you need to change the data when left arrow key is press
                
                ####################################################
            elif key == keyboard.Key.right:
                print('right')
                #################YOUR CODE HERE#####################
                #you need to change the data when right arrow key is press
                
                ####################################################
            else:
                pass
        except:
            print('Error')

    def on_release(self, key):
        #################YOUR CODE HERE#####################
        #you need to change the data back to 0 (stop) when key is release
        
        ####################################################
            #print('stop')
        if key == keyboard.Key.esc:
            # Stop listener
            os._exit(0)


if __name__ == '__main__':
    # Collect events until released
    ip = input('please input your master ip:')
    my_base_control = base_control(ip)
    with keyboard.Listener(on_press=my_base_control.on_press, on_release=my_base_control.on_release) as listener:
        listener.join()

    # ...or, in a non-blocking fashion:
    listener = keyboard.Listener(on_press=my_base_control.on_press, on_release=my_base_control.on_release)
    listener.start()
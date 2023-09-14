#!/usr/bin/env python
# coding: utf-8



from arg_robotics_tools import websocket_rosbridge as socket
from pynput import keyboard
import threading
import time
import os


class arm_control():
    def __init__(self, ip):
        self.ip = ip
        self.data = {'data':'0'}
        ##################YOUR CODE HERE###########
        #you need create a ros_socket connection to ip and port 9090 
        #and create a publisher by your ros_socket connection to topic named '/cmd_teleop', and its type is 'std_msgs/String'
        
        ###########################################  
        print('press w(+x), s(-x), a(+y), d(-y) ,z(+z), x(-z) ,n(gripper open) ,m(gripper close)')

    def on_press(self, key):
        ######YOUR CODE HERE#############
        #you need to change {'data':''} when a key is press
        #e.g. press a the data will be {'data':'a'} 
        try:
            if key == keyboard.KeyCode.from_char('w'):#arm will go +x    
                print('w', end='\r')
                
                

            elif key == keyboard.KeyCode.from_char('s'):#arm will go -x 
                print('s', end='\r')   
                
                
        
            elif key == keyboard.KeyCode.from_char('a'):#arm will go +y    
                print('a', end='\r')
                
                
            elif key == keyboard.KeyCode.from_char('d'):#arm will go -y    
                print('d', end='\r')
                
            
            elif key == keyboard.KeyCode.from_char('z'):#arm will go +z    
                print('z', end='\r')
                
                
            
            elif key == keyboard.KeyCode.from_char('x'):#arm will go -z    
                print('x', end='\r')
                
                
            
            
            elif key == keyboard.KeyCode.from_char('n'):#arm gripper will open   
                print('n', end='\r')
                
                
            
            elif key == keyboard.KeyCode.from_char('m'):#arm gripper will close
                print('m', end='\r')
                
                
        except:
            pass
        #############################
    def on_release(self, key):
        if key == keyboard.Key.esc:
            os._exit(0)


if __name__ == '__main__':
    ip = input('please input your robot ip:')
    my_arm_control = arm_control(ip)

    with keyboard.Listener(on_press=my_arm_control.on_press, on_release=my_arm_control.on_release) as listener:
        listener.join()

    listener = keyboard.Listener(on_press=my_arm_control.on_press,on_release=my_arm_control.on_release)
    listener.start()


    
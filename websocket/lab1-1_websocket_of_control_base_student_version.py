#!/usr/bin/env python
# coding: utf-8

# In[ ]:


from arg_robotics_tools import websocket_rosbridge as socket
from pynput import keyboard
import threading
import time
import os

# In[ ]:


data = {'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}


# In[ ]:


def key_publish():
    global data
    ###########code here #############
    #use socket.ros_socket() generate a connection to your rosbridge server
    #generate a publisher
    #topic name is 'cmd_vel_mux/input/teleop', type is 'geometry_msgs/Twist'
    ##################################
    
    while True:
    ###########code here #############
    #use pub to publish data with publisher 
    #you can change the publish frequency by changing number in sleep()
    ##################################

        time.sleep(5)

def on_press(key):
    ###code here ###
    #you need to change data when you push up down left right key
    # linear x do not exceed 0.5
    # angular z do not exceed 1 (if robot no rotate z need bigger)
    global data
    try:
        if key == keyboard.Key.up:
            print('up')
            
        elif key == keyboard.Key.down:
            print('down')
            
        elif key == keyboard.Key.left:
            print('left')
            
        elif key == keyboard.Key.right:
            print('right')
            
        else:
            pass
    except:
        print('Error')
    ####################
    
    
def on_release(key):
    #you need to change data to no key input when you release the key  
    global data
    ###code here###
    
    if key == keyboard.Key.esc:
        os._exit(0)
        


# In[ ]:


# Collect events until released
t = threading.Thread(target = key_publish)
t.start()
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()















#!/usr/bin/env python
# coding: utf-8

# In[1]:


from arg_robotics_tools import websocket_rosbridge as socket
from pynput import keyboard
import threading
import time
import os


# In[2]:


data = {'data':'0'}


# In[3]:
##################code here###########
 #IN LAB1-2 control arm you need to publish your String to topic named '/cmd_teleop', and its type is 'std_msgs/String'  
 #you need start a connect by take look at lab1 easy_publisher.py
 
print('press w(+x), s(-x), a(+y), d(-y) ,z(+z), x(-z) ,n(gripper open) ,m(gripper close)')

######################################

def on_press(key):
    ######code Here #############
    ##you need to change 'data':'' when a key is press
    # and use pub method to publish message by publisher 
    global data
    try:
        if key == keyboard.KeyCode.from_char('w'):#arm will go +x
            print('w',end='\r')
                

        elif key == keyboard.KeyCode.from_char('s'):#arm will go -x    
            print('s',end='\r')

    
        elif key == keyboard.KeyCode.from_char('a'):#arm will go +y    
            print('a',end='\r')

        
        elif key == keyboard.KeyCode.from_char('d'):#arm will go -y    
            print('d',end='\r')

        
        elif key == keyboard.KeyCode.from_char('z'):#arm will go +z    
            print('z',end='\r')


        elif key == keyboard.KeyCode.from_char('x'):#arm will go -z    
            print('x',end='\r')

        
        elif key == keyboard.KeyCode.from_char('n'):#arm gripper will open   
            print('n',end='\r')

        
        elif key == keyboard.KeyCode.from_char('m'):#arm gripper will close
            print('m',end='\r')


    except:
        pass
    #############################
def on_release(key):
    global data
    if key == keyboard.Key.esc:
        os._exit(0)

# In[ ]:


with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()


# ...or, in a non-blocking fashion:
listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()


    

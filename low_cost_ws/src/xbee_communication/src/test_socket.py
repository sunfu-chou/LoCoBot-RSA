#!/usr/bin/python3           # This is client.py file


# import socket

# HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
# PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

import socket
import rospy
from std_msgs.msg import Bool , Int32
from sensor_msgs.msg import NavSatFix
import datetime


class server_program(object):
    def __init__(self):
    # get the hostname
        self.host = socket.gethostname()
        self.port = 5000  # initiate port no above 1024

        self.server_socket = socket.socket()  # get instance
        # look closely. The bind() function takes tuple as argument
        self.server_socket.bind((self.host, self.port))  # bind host address and port together

        # configure how many client the server can listen simultaneously
        self.server_socket.listen(2)
        self.conn, self.address = self.server_socket.accept()  # accept new connection
        print("Connection from: " + str(self.address))
        self.sub_gps = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.cb_gps, queue_size=1)
        self.sub_auto = rospy.Subscriber("/auto_state", Bool, self.cb_auto, queue_size=1)
        self.sub_estop = rospy.Subscriber("/stop_state", Bool, self.cb_stop, queue_size=1)
        self.sub_drone_state = rospy.Subscriber("/drone/state", Int32, self.cb_uav, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1), self.cb_heartbeat)
        self.timer_1 = rospy.Timer(rospy.Duration(1), self.cb_gettime)
        self.timer_2 = rospy.Timer(rospy.Duration(1), self.cb_filldata)

        self.data = ""
        self.mode = 3
        self.teamid = "NYCU"


    def cb_heartbeat(self,event):
        # receive data stream. it won't accept data packet greater than 1024 bytes
        self.conn.send(self.data.encode())  # send data to the client

    def cb_gettime(self,event):
        self.now = datetime.datetime.now()
        self.year = str(self.now.year)[-2:]
        self.month = str(self.now.month)
        self.day = str(self.now.day)
        self.hour = str(self.now.hour)
        self.minute = str(self.now.minute)
        self.second = str(self.now.second)

    def cb_filldata(self,event):
        self.heartbeat_task()



    def cb_gps(self,msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def cb_auto(self,msg):
        self.auto = msg.data

    def cb_estop(self,msg):
        self.estop = msg.data
        if self.estop:
            self.mode = 3
        else:
            if self.auto:
                self.mode = 2
            else:
                self.mode = 1

    def cb_uav(self,msg):
        self.uav_mode = msg.data

    def checksum(self,sentence):
        len1 = len(sentence)

        ans = ord(sentence[0])
 
        for i in range(1,len1):
    
            # Traverse string to find the XOR
            ans = (ans ^ (ord(sentence[i])))

            ans = str(hex(ans))[-2:]

        return ans

    def heartbeat_task(self):
        sentence = "RXHRB," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+str(self.latitude)+",S,"+str(self.longitude)+ \
            ",E,"+ self.teamid + "," +str(self.mode)+","+str(self.uav_mode)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"

    def dock_task(self):
        sentence = "RXDOK," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+str(self.latitude)+",S,"+str(self.longitude)+ \
            ",E,"+ self.teamid + "," + 

    

    




if __name__ == '__main__':
    rospy.init_node("Server_program")
    Server_program = server_program()
    rospy.spin()
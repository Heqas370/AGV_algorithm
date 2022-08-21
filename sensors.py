#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
#from sensor_msgs.msg import Imu
import string
import socket
from socket import*
import time

address = ("172.17.10.173", 11411); # Defines server IP and port
client_socket = socket(AF_INET,SOCK_DGRAM) # Set up a socket


pub_pololu=[]
for i in range(8):
    pub_pololu.append(rospy.Publisher('AGV_driver/pololu/scan_'+str(i), Range, queue_size=10))

pub_encoder=[]
for i in range(2):
    pub_encoder.append(rospy.Publisher('AGV_driver/encoder_'+str(i), Range , queue_size=10))


rospy.init_node('AGV_driver', anonymous=False)

msg = Range()

def publishers():
     rate = rospy.Rate(1000) # 1 kHz
     while not rospy.is_shutdown():
        data = "Pololu"
        data_code = str.encode(data) # Parse "pololu" into UTF-8
        client_socket.sendto(data_code, address) # Send a request to server
        try:
            (rec_data, addr) = client_socket.recvfrom(10000) # Receive data from server
            parser = str(rec_data) # Parse rec_data into string
            split_data = parser.split(",")
            split_data = map(int, split_data)
            for i in range(8):
                for i, pub in enumerate(pub_pololu):
                    msg.range = split_data[i]
                    pub.publish(msg)
        except:
            pass
        data = "Encoder"
        data_code = str.encode(data) # Parse "TFMini" into UTF-8
        client_socket.sendto(data_code, address) # Send a request to server
        try:
            (rec_data, addr) = client_socket.recvfrom(1000) # Receive data from server
            parser = str(rec_data) # Parse rec_data into string
            split_data = parser.split(",")
            pars = map(int, split_data)
            rate_encoder = rospy.Rate(100) # 100 Hz
            for i, pub in enumerate(pub_encoder):
                msg.range = pars[i]  
                pub.publish(msg)
        except:
            pass
        rate_encoder.sleep()
        rate.sleep()
if __name__ == '__main__':
    try:
        publishers()

    except rospy.ROSInterruptException:
        pass


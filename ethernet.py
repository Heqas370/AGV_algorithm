import string
import socket
from socket import*
import time

address = ("169.254.72.130", 5000); # Defines server IP and port
client_socket = socket(AF_INET,SOCK_DGRAM) # Set up a socket

while(1):

    data = "Pololu"
    data_code = str.encode(data) # Parse "pololu" into UTF-8
    client_socket.sendto(data_code, address) # Send a request to server
    
    try:
        (rec_data, addr) = client_socket.recvfrom(1000) # Receive data from server
        parser = str(rec_data) # Parse rec_data into string
        split_data = parser.split(",")
        for i in split_data:
            print(i)
    except:
       pass
        
    data = "TFMini"
    data_code = str.encode(data) # Parse "TFMini" into UTF-8
    client_socket.sendto(data_code, address) # Send a request to server
    
    try:
        (rec_data, addr) = client_socket.recvfrom(1000) # Receive data from server
        parser = str(rec_data) # Parse rec_data into string
        split_data = parser.split(",")
        for i in split_data:
            print(i)
    except:
        pass
    
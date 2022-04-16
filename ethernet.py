import string
import socket
from socket import*
import time

address = ("169.254.72.130", 5000); # Defines server IP and port
client_socket = socket(AF_INET,SOCK_DGRAM) # Set up a socket

while(1):

    pololu = "Pololu"
    bytePololu = str.encode(pololu) # Parse "pololu" into UTF-8
    client_socket.sendto(bytePololu, address) # Send a request to server 
    
    try:
        (dataPololu, addr) = client_socket.recvfrom(2048) # Receive data from server
        parserPololu = str(dataPololu) # Parse rec_data into string
        splitPololu = parserPololu.split(",")
        for i in splitPololu:
            print("Pololu: ", i)
    
    except:
        pass
    
    time.sleep(2) # Delay before sending next command
    
    tfmini = "TFMini"
    byteTFMini = str.encode(tfmini)
    client_socket.sendto(byteTFMini, address)
    
    try:
        (dataTFMini, addr) = client_socket.recvfrom(2048)
        parserTFMini = str(dataTFMini)
        splitTFMini = parserTFMini.split(",")
        for i in splitTFMini:
            print("TFMini: ", i)
            
    except:
        pass
        
    time.sleep(2) # Delay before sending next command
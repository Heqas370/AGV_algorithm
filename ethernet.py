import string
import socket
from socket import*
import time

address = ("169.254.72.130", 5000);
client_socket = socket(AF_INET,SOCK_DGRAM)
#client_socket.settimeout(1)


while(1):
    j = 0
    start = time.time()
    data = "TFmini"
    xd = str.encode(data)
    z = client_socket.sendto(xd, address)
    (rec_data, addr) = client_socket.recvfrom(1000)
    parser = str(rec_data)
    tab = []
    tab.append(parser)
    data_split = parser.split(",")
    for i in data_split:
        print(i)
        end = time.time()
    print(end - start)
        
        
    
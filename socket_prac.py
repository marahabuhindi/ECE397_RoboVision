import socket, sys

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST, PORT))

while True:
    data,address = s.recvfrom(4096)
    print(data)
    
s.close()

        

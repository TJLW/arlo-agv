#!/usr/bin/env python



# If running on iOS with Pythonista....
# try:
#     import json
# except ImportError:
#     import simplejson as json



import socket
import sys

# def get_local_ip():
# 	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 	try:
# 		s.connect(('google.com', 80))
# 		ip = s.getsockname()[0]
# 		s.close()
# 	except:
# 		ip = 'N/A'
# 	return ip


## Main

print(sys.argv[1])


TCP_IP = '10.0.0.150'
TCP_PORT = 5005
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send(MESSAGE)
data = s.recv(BUFFER_SIZE)
s.close()

print "received data:", data

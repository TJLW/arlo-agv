#!/usr/bin/env python

import socket

def get_local_ip():
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	try:
		s.connect(('google.com', 80))
		ip = s.getsockname()[0]
		s.close()
	except:
		ip = 'N/A'
	return ip


# Python 2.7.15rc1 (default, Nov 12 2018, 14:31:15)
# [GCC 7.3.0] on linux2
# Type "help", "copyright", "credits" or "license" for more information.
# >>> import freenect
# >>> ctx = freenect.init()
# >>> dev_out = freenect.open_device(ctx, 1)
# Failed to open camera subdevice or it is not disabled.Failed to open motor subddevice or it is not disabled.Failed to open audio subdevice or it is not disabled.>>>
# >>> dev_out = freenect.open_device(ctx, 0)
# >>> freenect.set_tilt_degs(dev_out._ptr, 30)
# Traceback (most recent call last):
#   File "<stdin>", line 1, in <module>
# AttributeError: 'freenect.DevPtr' object has no attribute '_ptr'
# >>> freenect.set_tilt_degs(dev_out, 30)
# >>> freenect.set_tilt_degs(dev_out, 0)
#



print get_local_ip()
TCP_IP = get_local_ip()
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

while 1:
	conn, addr = s.accept()
	print 'Connection address:', addr

	data = conn.recv(BUFFER_SIZE)
	if not data: break
	print "received data:", data
	conn.send(data)  # echo
	conn.close()

import time

x = 0
f = open('saved-robot-state','w+')

while True:
	x += 1
	f.write('\rstate:' + str(x))
	time.sleep(2)

import time
from roboclaw_3 import Roboclaw

#Windows comport name
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80

for i in range(1):
	rc.ForwardM1(address,127)	#1/4 power forward
	time.sleep(2)
	
	rc.BackwardM1(address,0)	#Stopped
	rc.ForwardM2(address,0)		#Stopped
	time.sleep(2)
	

#fake the path of handle 3 book at different place
#Straight for 1 m, turn left for 90, go straight for one meter, turn right for 90


import serial
import time
from crc import crc



def TURN_LEFT(theta_current,theta_next):

	theta_threshold = 0.0698
	if(theta_next < theta_current):
		theta_change = 81000*(theta_next - theta_current)
	elif(theta_next > theta_current):
		theta_change = 81000*(theta_current - theta_next)

	theta_pu = int(theta_change)
	
	theta_pu_1 = "01" + "78" + (hex(-theta_pu)[2:]).zfill(8)
	theta_pu_1_crc = crc(theta_pu_1) 
	theta_pu_1_crc_hex = hex(theta_pu_1_crc)[2:]
	theta_pu_1_string = theta_pu_1 + theta_pu_1_crc_hex
	theta_pu_1_send = theta_pu_1_string.decode('hex')

	theta_pu_2 = "02" + "78" + (hex(-theta_pu)[2:]).zfill(8)
	theta_pu_2_crc = crc(theta_pu_2)
	theta_pu_2_crc_hex = hex(theta_pu_2_crc)[2:]
	theta_pu_2_string = theta_pu_2 + theta_pu_2_crc_hex
	theta_pu_2_send = theta_pu_2_string.decode('hex')	

	ser1.write(theta_pu_1_send)
	ser2.write(theta_pu_2_send)
	print "Finish Left Turn"
	

def TURN_RIGHT(theta_current,theta_next):

	theta_threshold = 0.0698
	if(theta_next > theta_current):
		theta_change = 81000*(theta_next - theta_current)
	elif(theta_next < theta_current):
		theta_change = 81000*(theta_current - theta_next)

	theta_pu = int(theta_change)

	theta_pu_1 = "01" + "78" + hex(-theta_pu & (2**32-1))[2:10]
	theta_pu_1_crc = crc(theta_pu_1) 
	theta_pu_1_crc_hex = hex(theta_pu_1_crc)[2:]
	theta_pu_1_string = theta_pu_1 + theta_pu_1_crc_hex
	theta_pu_1_send = theta_pu_1_string.decode('hex')

	theta_pu_2 = "02" + "78" + hex(-theta_pu & (2**32-1))[2:10]
	theta_pu_2_crc = crc(theta_pu_2)
	theta_pu_2_crc_hex = hex(theta_pu_2_crc)[2:]
	theta_pu_2_string = theta_pu_2 + theta_pu_2_crc_hex
	theta_pu_2_send = theta_pu_2_string.decode('hex')	

	ser1.write(theta_pu_1_send)
	ser2.write(theta_pu_2_send)
	print "Finish Right Turn"	


# go straight forward for x_change meter
def STRAIGHT(xy_current,xy_next):
	xy_threshold = 0.1 
	
	if(xy_next - xy_current >0):
		xy_change = 366936*(xy_next - xy_current)
	elif(xy_next - xy_current <0):
		xy_change = 366936*(xy_current - xy_next)

	xy_pu = int(xy_change)

	xy_pu_1 = "01" + "78" + (hex(xy_pu)[2:]).zfill(8)
	xy_pu_1_crc = crc(xy_pu_1)
	xy_pu_1_crc_hex = hex(xy_pu_1_crc)[2:]
	xy_pu_1_string = xy_pu_1 + xy_pu_1_crc_hex
	xy_pu_1_send = xy_pu_1_string.decode('hex')

	xy_pu_2 = "02" + "78" + hex(-xy_pu & (2**32-1))[2:10]
	xy_pu_2_crc = crc(xy_pu_2)
	xy_pu_2_crc_hex = hex(xy_pu_2_crc)[2:]
	xy_pu_2_string = xy_pu_2 + xy_pu_2_crc_hex
	xy_pu_2_send = xy_pu_2_string.decode('hex')	

	ser1.write(xy_pu_1_send)
	ser2.write(xy_pu_2_send)
	print "Finish Straight"

# go backforward for x_change meter
def BACKWARD(xy_current,xy_next):
	xy_threshold = 0.1 
	if(xy_next - xy_current < 0):
		xy_change = 366936*(xy_next - xy_current)
	elif(xy_next - xy_current > 0):
		xy_change = 366936*(xy_current - xy_next)

	xy_pu = int(xy_change)

	xy_pu_1 = "01" + "78" + hex(xy_pu & (2**32-1))[2:10]
	xy_pu_1_crc = crc(xy_pu_1)
	xy_pu_1_crc_hex = hex(xy_pu_1_crc)[2:]
	xy_pu_1_string = xy_pu_1 + xy_pu_1_crc_hex
	xy_pu_1_send = xy_pu_1_string.decode('hex')
    #this one is correct
	xy_pu_2 = "02" + "78" + (hex(-xy_pu)[2:]).zfill(8)
	xy_pu_2_crc = crc(xy_pu_2)
	xy_pu_2_crc_hex = hex(xy_pu_2_crc)[2:]
	xy_pu_2_string = xy_pu_2 + xy_pu_2_crc_hex
	xy_pu_2_send = xy_pu_2_string.decode('hex')	

	ser1.write(xy_pu_1_send)
	ser2.write(xy_pu_2_send)
	print "Finish Backward"

def theta_norm(t):
	if (t > 3.1415926):
		t -= (2 * 3.1415926)

	if (t < -3.1415926):
		t += (2 * 3.1415926)
	return t





if __name__ == '__main__':
	#Basic set up
	ser1 = serial.Serial('COM5', 19200) 
	ser2 = serial.Serial('COM9', 19200)
	ser1.write(b'\x01\x06\x00\x00\x00\x01\x48\x0A')
	ser2.write(b'\x02\x06\x00\x00\x00\x01\x48\x39')
	time.sleep(0.1)
	
	#this is for set up time
	time.sleep(5)
	
	#Move
	#Go to find book 1
	#this path is turn left 90 go staight for 1.8m, turn right for 90, go for 0.3m
	# straight for 1 m
	TURN_LEFT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,1.6)
	time.sleep(18)
	TURN_RIGHT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,0.3)
	time.sleep(7)
	#Go to find book 2
	#this path is turn right 90 go staight for 0.3m, turn left for 90
	TURN_RIGHT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,0.3)
	time.sleep(4)
	TURN_LEFT(0,1.57)
	time.sleep(10)
	#Go to find book 3
	#this path is turn right 90 go staight for 0.3m, turn left for 90
	TURN_RIGHT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,0.6)
	time.sleep(7)
	TURN_LEFT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,0.3)
	time.sleep(4)
	TURN_LEFT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,0.3)
	time.sleep(4)
	TURN_RIGHT(0,1.57)
	time.sleep(9)
	#Go back to start point
	TURN_RIGHT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,0.8)
	time.sleep(15) 
	TURN_RIGHT(0,1.57)
	time.sleep(5)
	STRAIGHT(0,0.6)
	time.sleep(8)
	TURN_RIGHT(0,3.14)
	time.sleep(10)




	ser1.close()
	ser2.close()
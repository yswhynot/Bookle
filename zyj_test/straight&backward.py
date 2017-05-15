#Straight for 0.2 m and backward for 0.2m

import serial
import time
from crc import crc



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

def action_state(x_current, x_next, y_current, y_next, theta_current, theta_next):
	#define position theta
	x_positive = 0
	x_negative = -3.1415926
	y_positive = -1.5707963
	y_negative = 1.5707963
	theta_threshold = 0.0698
	xy_threshold = 0.1 
	# if theta change
	if((theta_norm(theta_next - theta_current) > theta_threshold ) or (theta_norm(theta_next - theta_current) < -theta_threshold)):
		if((x_next - x_current > xy_threshold )or(x_next - x_current < -xy_threshold)or(y_next - y_current > xy_threshold )or(y_next - y_current < -xy_threshold)):
			return 
		elif(theta_norm(theta_current - theta_next) > theta_threshold):
			TURN_LEFT(theta_current,theta_next)
		elif(theta_norm(theta_next - theta_current) > theta_threshold):
			TURN_RIGHT(theta_current,theta_next)
	else:
		# if x change
		if((x_next - x_current > xy_threshold )or(x_next - x_current < -xy_threshold)):
			if((y_next - y_current > xy_threshold )or(y_next - y_current < -xy_threshold)):
				return
			elif((abs(theta_norm(x_positive - theta_current)) < theta_threshold) and (x_next > x_current)):
				STRAIGHT(x_current,x_next)
			elif((abs(theta_norm(x_positive - theta_current)) < theta_threshold) and (x_next < x_current)):
				BACKWARD(x_current,x_next)
			elif((abs(theta_norm(x_negative - theta_current)) < theta_threshold) and (x_next < x_current)):
				STRAIGHT(x_current,x_next)
			elif((abs(theta_norm(x_negative - theta_current)) < theta_threshold) and (x_next > x_current)):
				BACKWARD(x_current,x_next)
		# if y change
		elif((y_next - y_current > xy_threshold )or(y_next - y_current < -xy_threshold)):
			if((x_next - x_current > xy_threshold )or(x_next - x_current < -xy_threshold)):
				return
			elif((abs(theta_norm(y_positive - theta_current)) < theta_threshold) and (y_next > y_current)):
				STRAIGHT(y_current,y_next)
			elif((abs(theta_norm(y_positive - theta_current)) < theta_threshold) and (y_next < y_current)):
				BACKWARD(y_current,y_next)
			elif((abs(theta_norm(y_negative - theta_current)) < theta_threshold) and (y_next < y_current)):
				STRAIGHT(y_current,y_next)
			elif((abs(theta_norm(y_negative - theta_current)) < theta_threshold) and (y_next > y_current)):
				BACKWARD(y_current,y_next)
		print "Finish Action"



if __name__ == '__main__':
	#Basic set up
	ser1 = serial.Serial('COM5', 19200) 
	ser2 = serial.Serial('COM9', 19200)
	ser1.write(b'\x01\x06\x00\x00\x00\x01\x48\x0A')
	ser2.write(b'\x02\x06\x00\x00\x00\x01\x48\x39')
	time.sleep(0.1)
	#Move
	action_state(0,0.05,0,0,0,0)
	#this sleep cannot be deleted
	time.sleep(0.1)


	ser1.close()
	ser2.close()
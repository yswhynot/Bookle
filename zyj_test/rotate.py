import serial
import time
from crc import crc


def decode_wheelencoder(line):
	pu = " ".join(hex(ord(n))[2:] for n in line)	
	pu_addr = " ".join(hex(ord(n))[2:] for n in line[:1])
	pu_hex_1_51 = " ".join(hex(ord(n))[2:] for n in line[-2:-1])
	pu_hex_2_51 = " ".join(hex(ord(n))[2:] for n in line[-1:])
	pu_hex_3_51 = " ".join(hex(ord(n))[2:] for n in line[-4:-3])
	pu_hex_4_51 = " ".join(hex(ord(n))[2:] for n in line[-3:-2])
	pu_hex_51 = pu_hex_1_51.zfill(2) + pu_hex_2_51.zfill(2) + pu_hex_3_51.zfill(2) + pu_hex_4_51.zfill(2)
	pu_dec_51 = int(pu_hex_51,16)
	if pu_dec_51 > 0x7FFFFFFF:
		pu_dec_51 -= 0x100000000
	pu_meter_51 = float(pu_dec_51)/366936

	pu_hex_1_31 = " ".join(hex(ord(n))[2:] for n in line[29:30])
	pu_hex_2_31 = " ".join(hex(ord(n))[2:] for n in line[30:31])
	pu_hex_3_31 = " ".join(hex(ord(n))[2:] for n in line[27:28])
	pu_hex_4_31 = " ".join(hex(ord(n))[2:] for n in line[28:29])
	pu_hex_31 = pu_hex_1_31.zfill(2) + pu_hex_2_31.zfill(2) + pu_hex_3_31.zfill(2) + pu_hex_4_31.zfill(2)
	pu_dec_31 = int(pu_hex_31,16)
	if pu_dec_31 > 0x7FFFFFFF:
		pu_dec_31 -= 0x100000000

	print "Address:", pu_addr, "PU sum:", pu_dec_51, "PU:", pu_dec_31, "Passed meter:", pu_meter_51, "meter" 

	if(pu_dec_31 == 0):
		flag_move = 0
		print "Current state:", flag_move
	elif(pu_dec_31 is not "0"):
		flag_move = 1
		print "Current state:", flag_move

	#for drop packets
	if((pu_meter_51 > 30) or (pu_addr is not "1") or (pu_addr is not "2") or (pu_dec_31 < 0)):
		return None


def get_wheelencoder():
	ser1.write(b'\x01\x03\x00\x00\x00\x1A\xC4\x01')	
	line1 = ser1.read(51) 
	decode_wheelencoder(line1)
	ser2.write(b'\x02\x03\x00\x00\x00\x1A\xC4\x32')
	line2 = ser2.read(51)
	decode_wheelencoder(line2)


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
	action_state(-1.1040203,0.3789542,-2.3734583,-2.3734583,-3.13987643,-3.14)
	#this sleep cannot be deleted
	time.sleep(0.1)

	for i in range(5):
		get_wheelencoder()
		#These flush must be put here
		ser1.flushInput()
		ser1.flushOutput()
		ser2.flushInput()
		ser2.flushOutput()
	#just for testing
		time.sleep(1)

	ser1.close()
	ser2.close()











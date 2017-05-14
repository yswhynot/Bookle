import rospy
import tf
from geometry_msgs.msg import Point

import serial
import time
import math
import numpy as np

from crc import crc

class BookleBridge:
	def __init__(self):
		rospy.init_node('hardware_bridge', anonymous=True)
		
		self.motor_run = False
		self.current_point = (0, 0, 0)
		self.next_point = (0, 0, 0)

		self.ser_left = serial.Serial('/dev/ttyUSB1', 19200)
		self.ser_right = serial.Serial('/dev/ttyUSB2', 19200)
		rospy.loginfo('Serial connected\n')
		
		self.listener = tf.TransformListener()
		self.target_sub = rospy.Subscriber("/bookle/target_pose", Point, self.target_cb, queue_size = 1)
		self.current_sub = rospy.Subscriber("/bookle/current_pose/point", Point, self.current_cb, queue_size = 1)

	def target_cb(self, input):
		self.next_point = (input.x, input.y, input.z)

	def current_cb(self, input):
		self.current_point = (input.x, input.y, input.z)

	def tf_init(self):
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(0.75, -0.75, 0),
			tf.transformations.quaternion_from_euler(0, 0, 0),
			rospy.Time.now(),
			'base_footprint',
			'odom')

	def TURN_LEFT(self, theta_current, theta_next):

		theta_threshold = 0.0698
		theta_change = 81000*(theta_next - theta_current)

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

		self.ser_right.write(theta_pu_1_send)
		self.ser_left.write(theta_pu_2_send)
		print "Finish Left Turn"

	def TURN_RIGHT(self, theta_current, theta_next):

		theta_threshold = 0.0698
		theta_change = 81000*(theta_next - theta_current)

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

		self.ser_right.write(theta_pu_1_send)
		self.ser_left.write(theta_pu_2_send)

	# go straight forward for x_change meter
	def STRAIGHT(self, xy_current, xy_next):
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

		self.ser_left.write(xy_pu_2_send)
		self.ser_right.write(xy_pu_1_send)
		print "Finish Straight"

	# go backforward for x_change meter
	def BACKWARD(self, xy_current, xy_next):
		xy_threshold = 0.1 

		xy_change = 366936*(xy_next - xy_current)
		xy_pu = int(xy_change)

                print 'xy_change: '
                print xy_change

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

                print 'xy_pu_1_string: '
                print xy_pu_1_string
                # print 'xy_pu_2_send: '
                # print hex(xy_pu_2_send)
                # print 'xy_pu_1_send: '
                # print hex(xy_pu_1_send)

		self.ser_left.write(xy_pu_2_send)
		self.ser_right.write(xy_pu_1_send)
		print "Finish backward"

	def pu2meter(self, line):
		pu_addr = " ".join(hex(ord(n))[2:] for n in line[:1])
		if ((pu_addr is not "1") or (pu_addr is not "2")):
			return None

		pu_hex = [" ".join(hex(ord(n))[2:] for n in line[-2:-1]),
			" ".join(hex(ord(n))[2:] for n in line[-1:]),
			" ".join(hex(ord(n))[2:] for n in line[-4:-3]),
			" ".join(hex(ord(n))[2:] for n in line[-3:-2])]
		pu_hex_fill = pu_hex[0].zfill(2) + pu_hex[1].zfill(2) + pu_hex[2].zfill(2) + pu_hex[3].zfill(2)
		pu_dec = int(pu_hex_fill, 16)
		if pu_dec > 0x7FFFFFFF:
			pu_dec -= 0x100000000
		pu_meter = float(pu_dec) / 366936

		if(pu_meter > 30):
			return None

		pu_hex_set = [" ".join(hex(ord(n))[2:] for n in line[29:30]),
			" ".join(hex(ord(n))[2:] for n in line[30:31]),
			" ".join(hex(ord(n))[2:] for n in line[27:28]),
			" ".join(hex(ord(n))[2:] for n in line[28:29])]
		pu_hex_set_fill = pu_hex_set[0].zfill(2) + pu_hex_set[1].zfill(2) + pu_hex_set[2].zfill(2) + pu_hex_set[3].zfill(2)
		pu_dec_set = int(pu_hex_set_fill, 16)
		if pu_dec_set > 0x7FFFFFFF:
			pu_dec_set -= 0x100000000

		if(pu_dec_31 == 0):
			self.motor_run = False
		elif(pu_dec_31 is not "0"):
			self.motor_run = True

		return pu_meter

	def get_right_distance(self, prev_pu):
                # return (0.001, rospy.Time.now())
		self.ser_right.write(b'\x01\x03\x00\x00\x00\x1A\xC4\x01')
		line = self.ser_right.read(51)
		curr_pu = self.pu2meter(line)

		if curr_pu is None:
			return (0, rospy.Time.now())

                print("right: " + (prev_pu - curr_pu))
		return (prev_pu - curr_pu, rospy.Time.now())
		# return (0.01, rospy.Time.now())

	def get_left_distance(self, prev_pu):
                # return (0.001, rospy.Time.now())
		self.ser_left.write(b'\x02\x03\x00\x00\x00\x1A\xC4\x32')
		line = self.ser_left.read(51)
		curr_pu = self.pu2meter(line)

		if curr_pu is None:
			return (0, rospy.Time.now())

		return (prev_pu - curr_pu, rospy.Time.now())
		# return (0.01, rospy.Time.now())

	def theta_norm(self, t):
		if (t > 3.1415926):
			t -= (2 * 3.1415926)

		if (t < -3.1415926):
			t += (2 * 3.1415926)
		return t

	def action_state(self):
                # return
		if self.motor_run:
			return
		if self.next_point is (0.0, 0.0, 0.0):
			return

		x_current = self.current_point[0]
		x_next = self.next_point[0]
		y_current = self.current_point[1]
		y_next = self.next_point[1]
		theta_current = self.current_point[2]
		theta_next = self.next_point[2]
		
		print 'current: %s' % (self.current_point,)
		print 'next: %s' % (self.next_point,)

		#define position theta
		x_positive = 0
		x_negative = -3.1415926
		y_positive = -1.5707963
		y_negative = 1.5707963
		theta_threshold = 0.0698
		xy_threshold = 0.1 
		
		# if theta change
		if(abs(self.theta_norm(theta_next - theta_current)) > theta_threshold):
			if(self.theta_norm(theta_current - theta_next) > theta_threshold):
				self.TURN_LEFT(theta_current,theta_next)
			elif(self.theta_norm(theta_next - theta_current) > theta_threshold):
				self.TURN_RIGHT(theta_current,theta_next)
		else:
			# if x change
			if(abs(x_next - x_current) > xy_threshold):
				if((abs(self.theta_norm(x_positive - theta_current)) < theta_threshold) and (x_next > x_current)):
					self.STRAIGHT(x_current,x_next)
				elif((abs(self.theta_norm(x_positive - theta_current)) < theta_threshold) and (x_next < x_current)):
					self.BACKWARD(x_current,x_next)
				elif((abs(self.theta_norm(x_negative - theta_current)) < theta_threshold) and (x_next < x_current)):
					self.STRAIGHT(x_current,x_next)
				elif((abs(self.theta_norm(x_negative - theta_current)) < theta_threshold) and (x_next > x_current)):
					self.BACKWARD(x_current,x_next)
			# if y change
			elif((y_next - y_current > xy_threshold )or(y_next - y_current < -xy_threshold)):
				if((abs(self.theta_norm(y_positive - theta_current)) < theta_threshold) and (y_next > y_current)):
					self.STRAIGHT(y_current,y_next)
				elif((abs(self.theta_norm(y_positive - theta_current)) < theta_threshold) and (y_next < y_current)):
					self.BACKWARD(y_current,y_next)
				elif((abs(self.theta_norm(y_negative - theta_current)) < theta_threshold) and (y_next < y_current)):
					self.STRAIGHT(y_current,y_next)
				elif((abs(self.theta_norm(y_negative - theta_current)) < theta_threshold) and (y_next > y_current)):
					self.BACKWARD(y_current,y_next)

	def update_transform(self, prev_time, prev_dis):
		try:
			(trans, rot) = self.listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn('TF lookup fail\n')
			self.tf_init()
			return [[rospy.Time.now(), rospy.Time.now()], [0, 0]]

		x_trans = trans[0]
		y_trans = trans[1]
		euler = tf.transformations.euler_from_quaternion(rot)
		yaw = euler[2]

		# Calculate differetial drive params
		l = 0.43
		w = 0
		path_r = 0
		icc = (0, 0)
		(dl, tmp_tl) = self.get_left_distance(prev_dis[0])
		(dr, tmp_tr) = self.get_right_distance(prev_dis[1])
		vl = dl / (tmp_tl - prev_time[0]).to_sec()
		vr = dr / (tmp_tr - prev_time[1]).to_sec()
		w = (vr - vl) / l
		if vl != vr:
			path_r = l * (vl + vr) / (2 * (vr - vl))
		else:
			path_r = -1

		# Straight line motion
		if path_r == -1:
			trans = (trans[0] + (dl + dr) / 2 * math.cos(yaw), 
				trans[1] + (dl + dr) / 2 * math.sin(yaw), 
				0)
		else:
			icc = (x_trans - (path_r * math.sin(yaw)), y_trans + (path_r * math.cos(yaw)))

			time_dif = ((tmp_tl - prev_time[0]).to_sec() + (tmp_tr - prev_time[1]).to_sec()) / 2
			angle_dif = w * time_dif
			dif_vec = np.array([icc[0], icc[1], angle_dif])
			origin_vec = np.array([x_trans - icc[0], y_trans - icc[1], yaw])
			rot_mat = np.matrix([
				[math.cos(angle_dif), -math.sin(angle_dif), 0],
				[math.sin(angle_dif), math.cos(angle_dif), 0],
				[0, 0, 1]])
			result = np.add(np.dot(rot_mat, origin_vec), dif_vec).tolist()

			trans = (result[0][0], result[0][1], 0)
			rot = tf.transformations.quaternion_from_euler(0, 0, result[0][2])
		
		self.action_state()

		br = tf.TransformBroadcaster()
		br.sendTransform(trans, rot, rospy.Time.now(), 'base_footprint', 'odom')

		print [dl + prev_dis[0], dr + prev_dis[1]]
		return [[tmp_tl, tmp_tr], [dl + prev_dis[0], dr + prev_dis[1]]]

	def clear_serial(self):
		self.ser_right.flushInput()
		self.ser_right.flushOutput()
		self.ser_left.flushInput()
		self.ser_left.flushOutput()

	def start(self):
		rospy.loginfo('Hardware bridge started :)\n')
		rate = rospy.Rate(10)

		time_array = [rospy.Time.now(), rospy.Time.now()]
		dis_array = [0, 0]

		while not rospy.is_shutdown():
			[time_array, dis_array] = self.update_transform(time_array, dis_array)

			self.clear_serial()		
			rate.sleep()

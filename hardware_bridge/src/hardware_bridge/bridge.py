import rospy
import tf

import serial
import time
import math
import numpy as np

def tf_init():
	br = tf.TransformBroadcaster()
	br.sendTransform(
		(0.75, -0.75, 0),
		tf.transformations.quaternion_from_euler(0, 0, 0),
		rospy.Time.now(),
		'base_footprint',
		'odom')

def init():
	rospy.init_node('hardware_bridge', anonymous=True)

def init_serial(ser_left, ser_right):
	ser_left.write(b'\x01\x06\x00\x00\x00\x01\x48\x0A')
	ser_right.write(b'\x02\x06\x00\x00\x00\x01\x48\x39')
	rospy.loginfo('Motor set up done\n')

def action_left(ser_left):
	ser_left.write(b'\x01\x78\x00\x06\x1A\x80\x4B\x01')

def action_right(ser_right):
	ser_right.write(b'\x02\x78\xFF\xF9\xE5\x80\x0A\xE6')

def parse_pu(line):
	pu_addr = " ".join(hex(ord(n))[2:] for n in line[:1])
	pu_hex = [" ".join(hex(ord(n))[2:] for n in line[-2:-1]),
		" ".join(hex(ord(n))[2:] for n in line[-1:]),
		" ".join(hex(ord(n))[2:] for n in line[-4:-3]),
		" ".join(hex(ord(n))[2:] for n in line[-3:-2])]
	pu_hex_fill = pu_hex[0].zfill(2) + pu_hex[1].zfill(2) + pu_hex[2].zfill(2) + pu_hex[3].zfill(2)
	pu_dec = int(pu_hex_fill, 16)
	pu_meter = float(pu_dec / 366936)
	return pu_meter

def get_left_distance(ser_left):
	ser_left.write(b'\x01\x03\x00\x00\x00\x1A\xC4\x01')
	line = ser_left.read(51)

	return (parse_pu(line), rospy.Time.now())
	# return (0.01, rospy.Time.now())

def get_right_distance(ser_right):
	ser_right.write(b'\x02\x03\x00\x00\x00\x1A\xC4\x32')
	line = ser_right.read(51)

	return (parse_pu(line), rospy.Time.now())
	# return (0.01, rospy.Time.now())

def update_transform(listener, tl, tr, ser_left, ser_right):
	try:
		(trans, rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		rospy.logwarn('TF lookup fail\n')
		tf_init()
		return (rospy.Time.now(), rospy.Time.now())

	x_trans = trans[0]
	y_trans = trans[1]
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[2]

	# Calculate differetial drive params
	l = 0.43
	w = 0
	path_r = 0
	icc = (0, 0)
	(dl, tmp_tl) = get_left_distance(ser_left)
	(dr, tmp_tr) = get_right_distance(ser_right)
	vl = dl / (tmp_tl - tl).to_sec()
	vr = dr / (tmp_tr - tr).to_sec()
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

		time_dif = ((tmp_tl - tl).to_sec() + (tmp_tr - tr).to_sec()) / 2
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
	
	action_left(ser_left)
	action_right(ser_right)

	br = tf.TransformBroadcaster()
	br.sendTransform(trans, rot, rospy.Time.now(), 'base_footprint', 'odom')

	return [tmp_tl, tmp_tr]

def clear_serial(ser_left, ser_right):
	ser_left.flushInput()
	ser_left.flushOutput()
	ser_right.flushInput()
	ser_right.flushOutput()

def start():
	rospy.loginfo('Hardware bridge started :)\n')
	init()
	ser_left = serial.Serial('/dev/ttyUSB2', 19200)
	ser_right = serial.Serial('/dev/ttyUSB1', 19200)
	rospy.loginfo('Serial connected\n')
	init_serial(ser_left, ser_right)

	rate = rospy.Rate(10)
	listener = tf.TransformListener()

	tl = rospy.Time.now()
	tr = rospy.Time.now()
	time_array = [tl, tr]

	while not rospy.is_shutdown():
		time_array = update_transform(listener, tl, tr, ser_left, ser_right)
		tl = time_array[0]
		tr = time_array[1]

		clear_serial(ser_left, ser_right)		
		rate.sleep()

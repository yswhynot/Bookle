import rospy
import tf

import time
import math

def init():
	rospy.init_node('hardware_bridge', anonymous=True)

	br = tf.TransformBroadcaster()
	br.sendTransform(
		(0.75, -0.75, 0),
		tf.transformations.quaternion_from_euler(0, 0, 0),
		rospy.Time.now(),
		'base_footprint',
		'odom')

def get_left_distance():
# TODO: return distance in meter in 100ms
	return (0.01, rospy.Time.now())

def get_right_distance():
	return (0.01, rospy.Time.now())

def update_transform(listener, tl, tr):
	try:
		(trans, rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print('TF lookup fail\n')
		break

	x_trans = trans[0]
	y_trans = trans[1]
	x_rot = rot[0]
	y_rot = rot[1]
	z_rot = rot[2]
	w_rot = rot[3]

	# Calculate differetial drive params
	l = 0.43
	angle = 0
	path_r = 0
	icc = (0, 0)
	(dl, tmp_tl) = get_left_distance
	(dr, tmp_tr) = get_right_distance
	vl = dl / (tmp_tl - tl)
	vr = dr / (tmp_tr - tr)
	angle = (vr - vl) / l
	if dl != dr:
		path_r = l * (vl + vr) / (2 * (vr - vl))
	else:
		path_r = -1



	return (tmp_tl, tmp_tr)


def start():
	print('Hardware bridge started :)\n')
	init()

	rate = rospy.Rate(10)
	listener = tf.TransformListener()

	tl = rospy.Time.now()
	tr = rospy.Time.now()

	while not rospy.is_shutdown():
		(tl, tr) = update_transform(listener, tl, tr)
		rate.sleep()
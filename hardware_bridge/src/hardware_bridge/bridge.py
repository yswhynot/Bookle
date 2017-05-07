import rospy

def init():
	rospy.init_node('hardware_bridge', anonymous=True)

def start():
	print('Hardware bridge started :)\n')
	init()

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep()
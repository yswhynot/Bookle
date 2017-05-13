import rospy
from bookle_sys_manager.srv import * 
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import RPi.GPIO as GPIO
import time

state = 'wait'

def cam_on():
	cam_serv_name = '/camera/start_capture'
	rospy.wait_for_service(cam_serv_name)
	cam_serv = rospy.ServiceProxy(cam_serv_name, Empty)
	res = cam_serv()

def cam_off():
	cam_serv_name = '/camera/stop_capture'
	rospy.wait_for_service(cam_serv_name)
	cam_serv = rospy.ServiceProxy(cam_serv_name, Empty)
	res = cam_serv()

def get_goal(code):
	code_map = {
		# [x, y, z, x, y, z, w]
		'12132054': [60, 40, 0, 0, 0, 0, 1],
		'98765432': [30, 40, 0, 0, 0, 0, 1]
	}
	if code not in code_map:
		return none

	goal = PoseStamped()
	goal.header.stamp = rospy.Time.now()
	goal.pose.position.x = code_map[code][0]
	goal.pose.position.y = code_map[code][1]
	goal.pose.position.z = code_map[code][2]
	goal.pose.orientation.x = code_map[code][3]
	goal.pose.orientation.y = code_map[code][4]
	goal.pose.orientation.z = code_map[code][5]
	goal.pose.orientation.w = code_map[code][6]

	return goal

def barcode_cb(input):
	rospy.loginfo('Barcode received!\n')
	goal_pub = rospy.Publisher('/bookle/goal_pos', PoseStamped, queue_size=1)
	code = input.data
	goal = get_goal(code)
	if goal is not None:
		goal_pub.publish(goal)
		cam_off()

		global state
		state = 'planning'
	else:
		rospy.loginfo('Barcode not in record\n')

def path_cb(input):
	rospy.loginfo('Path received!\n')
	global state
	state = 'running'

def init_goal():
	goal = PoseStamped()	
	goal.header.stamp = rospy.Time.now()
	goal.pose.position.x = 10
	goal.pose.position.y = 10
	goal.pose.position.z = 0
	goal.pose.orientation.x = 0
	goal.pose.orientation.y = 0
	goal.pose.orientation.z = 0
	goal.pose.orientation.w = 1

	goal_pub = rospy.Publisher('/bookle/goal_pos', PoseStamped, queue_size=1)
	goal_pub.publish(goal)

def check_button():
	if GPIO.input(25):
		init_goal()

def init_io():
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(18, GPIO.OUT)
	GPIO.setup(23, GPIO.OUT)
	GPIO.setup(24, GPIO.OUT)
	GPIO.setup(25, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

def init_ros():
	rospy.init_node('sys_manager', anonymous = True)

	rospy.Subscriber('/bookle/barcode', String, barcode_cb)
	rospy.Subscriber('/bookle/planned_path', Path, path_cb)
	cam_on()

	global state
	state = 'wait'

def led_on_red():
	GPIO.output(23, GPIO.LOW)
	GPIO.output(24, GPIO.LOW)
	GPIO.output(18, GPIO.HIGH)
	time.sleep(0.5)

def led_on_blue():
	GPIO.output(18, GPIO.LOW)
	GPIO.output(24, GPIO.LOW)
	GPIO.output(23, GPIO.HIGH)
	time.sleep(0.5)

def led_on_green():
	GPIO.output(18, GPIO.LOW)
	GPIO.output(23, GPIO.LOW)
	GPIO.output(24, GPIO.HIGH)
	time.sleep(0.5)

def led_blink_red():
	GPIO.output(23, GPIO.LOW)
	GPIO.output(24, GPIO.LOW)

	GPIO.output(18, GPIO.HIGH)
	time.sleep(0.25)
	GPIO.output(18, GPIO.LOW)
	time.sleep(0.25)

def led_blink_blue():
	GPIO.output(18, GPIO.LOW)
	GPIO.output(24, GPIO.LOW)

	GPIO.output(23, GPIO.HIGH)
	time.sleep(0.25)
	GPIO.output(23, GPIO.LOW)
	time.sleep(0.25)

def led_blink_green():
	GPIO.output(18, GPIO.LOW)
	GPIO.output(23, GPIO.LOW)

	GPIO.output(24, GPIO.HIGH)
	time.sleep(0.25)
	GPIO.output(24, GPIO.LOW)
	time.sleep(0.25)

def led_flash():
	GPIO.output(18, GPIO.HIGH)
	GPIO.output(23, GPIO.LOW)
	GPIO.output(24, GPIO.LOW)
	time.sleep(0.15)

	GPIO.output(18, GPIO.LOW)
	GPIO.output(23, GPIO.HIGH)
	GPIO.output(24, GPIO.LOW)
	time.sleep(0.15)

	GPIO.output(18, GPIO.LOW)
	GPIO.output(23, GPIO.LOW)
	GPIO.output(24, GPIO.HIGH)
	time.sleep(0.15)

def led_off():
	GPIO.output(18, GPIO.LOW)
	GPIO.output(23, GPIO.LOW)
	GPIO.output(24, GPIO.LOW)
	time.sleep(0.5)

actions = {
	'none': led_off,
	'wait': led_blink_red,
	'planning': led_blink_green,
	'running': led_on_green,
	'finish': led_flash,
	'error': led_on_red
}

def start():
	rospy.loginfo('System manager started :)\n')
	init_io()
	init_ros()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		actions[state]()
		rate.sleep()

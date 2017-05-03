import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

state = 'wait'

def init_io():
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(18, GPIO.OUT)
	GPIO.setup(23, GPIO.OUT)
	GPIO.setup(24, GPIO.OUT)

def init_ros():
	rospy.init_node('sys_manager', anonymous=True)

	cam_serv_name = '/camera/start_capture'
	rospy.wait_for_service(cam_serv_name)
	cam_serv = rospy.ServiceProxy(cam_serv_name)
	res = cam_serv()
	
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
	'goal': led_flash,
	'error': led_on_red
}

def start():
	print('System manager started :)\n')
	init_io()
	init_ros()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		actions[state]()
		rate.sleep()
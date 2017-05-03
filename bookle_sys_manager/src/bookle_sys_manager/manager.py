import RPi.GPIO as GPIO
import time

state = 'none'

def init_io():
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(18, GPIO.OUT)
	GPIO.setup(23, GPIO.OUT)
	GPIO.setup(24, GPIO.OUT)
	state = 'goal'

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
	time.sleep(0.1, GPIO.HIGH)

	GPIO.output(18, GPIO.LOW)
	GPIO.output(23, GPIO.HIGH)
	GPIO.output(24, GPIO.LOW)
	time.sleep(0.1)

	GPIO.output(18, GPIO.LOW)
	GPIO.output(23, GPIO.LOW)
	GPIO.output(24, GPIO.HIGH)
	time.sleep(0.1)

	GPIO.output(18, GPIO.HIGH)
	GPIO.output(23, GPIO.HIGH)
	GPIO.output(24, GPIO.HIGH)
	time.sleep(0.2)

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

	while 1:
		actions[state]()
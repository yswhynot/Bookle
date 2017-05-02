import RPi.GPIO as GPIO
import time

def init_io():
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(18, GPIO.OUT)

def led_blink():
	while 1:
		GPIO.output(18, GPIO.HIGH)
		time.sleep(0.5)
		GPIO.output(18, GPIO.LOW)
		time.sleep(0.5)

def start():
    print('System manager started\n')
    init_io()
    led_blink()

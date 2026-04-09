import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)  # IN3
GPIO.setup(6, GPIO.OUT)   # IN4
GPIO.setup(13, GPIO.OUT)  # ENB

GPIO.output(13, 1)        # ENABLE FULL

print("DIR 1")
GPIO.output(5, 1)
GPIO.output(6, 0)
time.sleep(3)

print("DIR 2")
GPIO.output(5, 0)
GPIO.output(6, 1)
time.sleep(3)

GPIO.cleanup()


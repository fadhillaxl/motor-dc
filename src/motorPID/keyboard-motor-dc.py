import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import threading

# ================= GPIO SETUP =================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ================= MOTOR CLASS =================
class Motor:
    def __init__(self, en, in1, in2, pwm_freq=800, inverted=False, accel_step=3):
        self.en = en
        self.in1 = in1
        self.in2 = in2
        self.inverted = inverted

        self.current_speed = 0
        self.target_speed = 0
        self.forward = True
        self.accel_step = accel_step
        self.running = True

        GPIO.setup(en, GPIO.OUT)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)

        self.pwm = GPIO.PWM(en, pwm_freq)
        self.pwm.start(0)

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _set_dir(self):
        fwd = not self.forward if self.inverted else self.forward
        GPIO.output(self.in1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW if fwd else GPIO.HIGH)

    def set(self, speed, forward=True):
        self.target_speed = speed
        self.forward = forward
        self._set_dir()

    def stop(self):
        self.target_speed = 0

    def _loop(self):
        while self.running:
            if self.current_speed < self.target_speed:
                self.current_speed += self.accel_step
            elif self.current_speed > self.target_speed:
                self.current_speed -= self.accel_step

            self.current_speed = max(0, min(100, self.current_speed))
            self.pwm.ChangeDutyCycle(self.current_speed)
            time.sleep(0.03)

    def cleanup(self):
        self.running = False
        self.thread.join()
        self.pwm.stop()
        GPIO.output(self.in1, 0)
        GPIO.output(self.in2, 0)

# ================= MOTOR SETUP =================
motor_a = Motor(en=18, in1=23, in2=24)
motor_b = Motor(en=13, in1=5, in2=6, inverted=True)

# ================= SPEED =================
BASE_SPEED = 50
SPEED_STEP = 5
speed = BASE_SPEED

# ================= KEYBOARD =================
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

last_input = time.time()

def watchdog():
    global last_input
    while True:
        if time.time() - last_input > 0.4:
            motor_a.stop()
            motor_b.stop()
        time.sleep(0.1)

threading.Thread(target=watchdog, daemon=True).start()

# ================= UI =================
print("""
=== ADVANCED DUAL MOTOR CONTROL ===

Motor B : ↑ ↓  or  W S
Motor A : ← →  or  A D

+ / -   Speed
SPACE   Stop
Q       Quit
""")

# ================= MAIN LOOP =================
try:
    while True:
        key = get_key()
        last_input = time.time()

        if key in ('\x1b[A', 'w'):
            motor_b.set(speed, True)

        elif key in ('\x1b[B', 's'):
            motor_b.set(speed, False)

        elif key in ('\x1b[C', 'd'):
            motor_a.set(speed, True)

        elif key in ('\x1b[D', 'a'):
            motor_a.set(speed, False)

        elif key == '+':
            speed = min(90, speed + SPEED_STEP)
            print("Speed:", speed)

        elif key == '-':
            speed = max(20, speed - SPEED_STEP)
            print("Speed:", speed)

        elif key == ' ':
            motor_a.stop()
            motor_b.stop()

        elif key.lower() == 'q':
            break

except KeyboardInterrupt:
    pass

finally:
    motor_a.cleanup()
    motor_b.cleanup()
    GPIO.cleanup()
    print("\nGPIO cleaned up")

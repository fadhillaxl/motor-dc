import RPi.GPIO as GPIO
import time

# ================== PIN CONFIG (BCM) ==================
# Motor A
ENA, IN1, IN2 = 18, 23, 24

# Motor B (IN4 diganti GPIO 6)
ENB, IN3, IN4 = 13, 5, 6

# ================== TUNING ==================
PWM_FREQ = 800      # Hz
RAMP_STEP = 2
RAMP_DELAY = 0.03

print("=== L298N DUAL MOTOR TEST (FIXED DIR) ===")
print("Ctrl+C to stop\n")

# ================== GPIO SETUP ==================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [ENA, IN1, IN2, ENB, IN3, IN4]:
    GPIO.setup(pin, GPIO.OUT)

# ================== PWM SETUP ==================
pwm_a = GPIO.PWM(ENA, PWM_FREQ)
pwm_b = GPIO.PWM(ENB, PWM_FREQ)
pwm_a.start(0)
pwm_b.start(0)

# ================== HELPER ==================
def ramp_pwm(pwm, start, end):
    step = RAMP_STEP if start < end else -RAMP_STEP
    for dc in range(start, end + step, step):
        pwm.ChangeDutyCycle(max(0, min(100, dc)))
        time.sleep(RAMP_DELAY)

# ================== MOTOR A ==================
def motor_a_forward(speed):
    GPIO.output(IN1, 1)
    GPIO.output(IN2, 0)
    ramp_pwm(pwm_a, 0, speed)

def motor_a_reverse(speed):
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 1)
    ramp_pwm(pwm_a, 0, speed)

def motor_a_stop():
    ramp_pwm(pwm_a, 30, 0)
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)

# ================== MOTOR B (DIR FIXED) ==================
def motor_b_forward(speed):
    # DIBALIK dari standar
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 1)
    ramp_pwm(pwm_b, 0, speed)

def motor_b_reverse(speed):
    GPIO.output(IN3, 1)
    GPIO.output(IN4, 0)
    ramp_pwm(pwm_b, 0, speed)

def motor_b_stop():
    ramp_pwm(pwm_b, 50, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)

# ================== MAIN LOOP ==================
try:
    while True:
        print("\n--- MOTOR A ---")
        print("Forward")
        motor_a_forward(30)
        time.sleep(2)

        print("Stop")
        motor_a_stop()
        time.sleep(1)

        print("Reverse")
        motor_a_reverse(30)
        time.sleep(2)

        print("Stop")
        motor_a_stop()
        time.sleep(2)

        print("\n--- MOTOR B ---")
        print("Forward")
        motor_b_forward(50)
        time.sleep(2)

        print("Stop")
        motor_b_stop()
        time.sleep(1)

        print("Reverse")
        motor_b_reverse(50)
        time.sleep(2)

        print("Stop")
        motor_b_stop()
        time.sleep(3)

        print("\n==============================")

except KeyboardInterrupt:
    print("\nStopping motors...")

finally:
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    print("GPIO cleaned up!")

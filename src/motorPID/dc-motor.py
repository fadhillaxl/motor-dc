import RPi.GPIO as GPIO
import time

# =========================
# CONFIG
# =========================
RPWM_PIN = 18   # GPIO18 (Pin 12)
LPWM_PIN = 19   # GPIO19 (Pin 35)

PWM_FREQ = 1000  # 1 kHz (safe for BTS7960 / HW-039)

# =========================
# GPIO SETUP
# =========================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(RPWM_PIN, GPIO.OUT)
GPIO.setup(LPWM_PIN, GPIO.OUT)

rpwm = GPIO.PWM(RPWM_PIN, PWM_FREQ)
lpwm = GPIO.PWM(LPWM_PIN, PWM_FREQ)

rpwm.start(0)
lpwm.start(0)

print("✅ GPIO initialized")
print(f"RPWM = GPIO {RPWM_PIN}, LPWM = GPIO {LPWM_PIN}")
print(f"PWM Frequency = {PWM_FREQ} Hz")

# =========================
# MOTOR CONTROL FUNCTIONS
# =========================
def stop_motor():
    print("🛑 Motor STOP")
    rpwm.ChangeDutyCycle(0)
    lpwm.ChangeDutyCycle(0)

def forward(speed):
    """
    speed: 0–100
    """
    print(f"➡️  Forward | Speed: {speed}%")
    lpwm.ChangeDutyCycle(0)
    rpwm.ChangeDutyCycle(speed)

def reverse(speed):
    """
    speed: 0–100
    """
    print(f"⬅️  Reverse | Speed: {speed}%")
    rpwm.ChangeDutyCycle(0)
    lpwm.ChangeDutyCycle(speed)

# =========================
# TEST SEQUENCE (DEBUG)
# =========================
try:
    print("\n=== MOTOR DEBUG TEST START ===\n")

    print("Forward 30%")
    forward(30)
    time.sleep(3)

    stop_motor()
    time.sleep(2)

    print("Forward 70%")
    forward(70)
    time.sleep(3)

    stop_motor()
    time.sleep(2)

    print("Reverse 40%")
    reverse(40)
    time.sleep(3)

    stop_motor()
    time.sleep(2)

    print("\n✅ TEST COMPLETE")

except KeyboardInterrupt:
    print("\n⚠️ Interrupted by user")

finally:
    stop_motor()
    rpwm.stop()
    lpwm.stop()
    GPIO.cleanup()
    print("🧹 GPIO cleaned up")

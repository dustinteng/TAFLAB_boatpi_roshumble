import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
servo_pin = 24  # GPIO 25 for the servo

# Set up GPIO output for the servo
GPIO.setup(servo_pin, GPIO.OUT)

# Set up PWM on the servo pin
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency (20 ms period)

# Start PWM with 0% duty cycle (servo in neutral position)
pwm.start(0)

def set_angle(pwm, angle):
    # Convert angle to duty cycle
    duty = 2 + (angle / 18)  # Map 0-180° to 2-12% duty cycle
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Wait for the servo to move

try:
    while True:
        # Move the servo to different angles
        angle = 0
        set_angle(pwm, angle)
        print(f"Servo moved to {angle}°")
        time.sleep(1)

        angle = 90
        set_angle(pwm, angle)
        print(f"Servo moved to {angle}°")
        time.sleep(1)

        angle = 180
        set_angle(pwm, angle)
        print(f"Servo moved to {angle}°")
        time.sleep(1)

        angle = 270
        set_angle(pwm, angle)
        print(f"Servo moved to {angle}°")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Cleanup
    pwm.stop()
    GPIO.cleanup()

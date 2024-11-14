import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
servo_pin_1 = 23  # GPIO 23 for first servo
servo_pin_2 = 25  # GPIO 25 for second servo

# Set up GPIO outputs for each servo
GPIO.setup(servo_pin_1, GPIO.OUT)
GPIO.setup(servo_pin_2, GPIO.OUT)

# Set up PWM on each servo pin
pwm1 = GPIO.PWM(servo_pin_1, 50)  # 50 Hz frequency (20 ms period)
pwm2 = GPIO.PWM(servo_pin_2, 50)  # 50 Hz frequency (20 ms period)

# Start PWM with 0% duty cycle (servos in neutral position)
pwm1.start(0)
pwm2.start(0)

def set_angle(pwm, angle):
    # Convert angle to duty cycle
    duty = 2 + (angle / 18)  # Map 0-180° to 2-12% duty cycle
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Wait for the servo to move

try:
    while True:
        # Move the first servo to different angles
        angle = 0
        set_angle(pwm1, angle)
        print(f"Servo 1 moved to {angle}°")
        set_angle(pwm2, angle)
        print(f"Servo 2 moved to {angle}°")
        time.sleep(1)

        angle = 90
        set_angle(pwm1, angle)
        print(f"Servo 1 moved to {angle}°")
        set_angle(pwm2, angle)
        print(f"Servo 2 moved to {angle}°")
        time.sleep(1)

        angle = 180
        set_angle(pwm1, angle)
        print(f"Servo 1 moved to {angle}°")
        set_angle(pwm2, angle)
        print(f"Servo 2 moved to {angle}°")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Cleanup
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData, CalibrationData  # Still subscribed, if needed
import RPi.GPIO as GPIO
import pigpio
import time
from queue import Queue
import threading
import json
import os
from std_msgs.msg import Float32

class BoatControlNode(Node):
    def __init__(self):
        super().__init__('boat_control_node')

        # GPIO setup for each servo
        self.RUDDER_PIN = 18
        self.SAIL_PIN = 12
        self.ESC_PIN = 13
        
        # Initialize pigpio for each servo
        self.rudder_pwm = pigpio.pi('localhost', 8888)
        self.rudder_pwm.set_mode(self.RUDDER_PIN, pigpio.OUTPUT)
        self.rudder_pwm.set_PWM_frequency(self.RUDDER_PIN, 50)
        
        self.sail_pwm = pigpio.pi()
        self.sail_pwm.set_mode(self.SAIL_PIN, pigpio.OUTPUT)
        self.sail_pwm.set_PWM_frequency(self.SAIL_PIN, 50)
        
        self.esc_pwm = pigpio.pi()
        self.esc_pwm.set_mode(self.ESC_PIN, pigpio.OUTPUT)
        self.esc_pwm.set_PWM_frequency(self.ESC_PIN, 50)
        
        # Initial positions/targets
        self.currentSailPosition = 90    # Commanded value in degrees (for sail: 0 to 180)
        self.currentRudderPosition = 0  # Commanded value in degrees (for rudder: -90 to 90)
        self.targetSailPos = 0
        self.targetRudderPos = 0
        self.motorSpeed = 0

        # Load duty-cycle limits from config file
        self.load_config()  # Sets: self.minDutyRudder, self.midDutyRudder, self.maxDutyRudder,
                             #       self.minDutySail, self.midDutySail, self.maxDutySail

        self.currentSailPublisher = self.create_publisher(Float32, '/currentSailPos', 10)
        
        # Queue for servo commands and worker thread
        self.command_queue = Queue()
        self.worker_thread = threading.Thread(target=self.process_commands)
        self.worker_thread.daemon = True
        self.worker_thread.start()

        # Subscribers to listen to control and (optionally) calibration commands
        self.subscription = self.create_subscription(
            ControlData,
            '/boatcontrol',
            self.control_callback,
            10
        )
        
        # Calibration subscription (if needed)
        self.calibration_subscription = self.create_subscription(
            CalibrationData,
            '/calibration',
            self.calibration_callback,
            10
        )

        self.get_logger().info("BoatControlNode initialized and listening to /boatcontrol and /calibration")

    def load_config(self):
        """Load servo duty-cycle limits from config.json."""
        config_path = os.path.join(os.path.dirname(__file__), "config.json")
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
            servo_config = config.get("servo_config", {})
            self.minDutyRudder = servo_config.get("minDutyRudder", 1041)
            self.midDutyRudder = servo_config.get("midDutyRudder", 1544)
            self.maxDutyRudder = servo_config.get("maxDutyRudder", 2027)
            self.minDutySail   = servo_config.get("minDutySail", 1137)
            self.midDutySail   = servo_config.get("midDutySail", 1642)
            self.maxDutySail   = servo_config.get("maxDutySail", 2107)
            self.get_logger().info(f"Config loaded: Rudder [{self.minDutyRudder}, {self.midDutyRudder}, {self.maxDutyRudder}], "
                                   f"Sail [{self.minDutySail}, {self.midDutySail}, {self.maxDutySail}]")
        else:
            # Use default values if config file is missing
            self.minDutyRudder = 1041
            self.midDutyRudder = 1544
            self.maxDutyRudder = 2027
            self.minDutySail   = 1137
            self.midDutySail   = 1642
            self.maxDutySail   = 2107
            self.get_logger().warn("Config file not found. Using default servo duty values.")

    def control_callback(self, msg):
        self.targetRudderPos = msg.servo_rudder
        self.targetSailPos = msg.servo_sail
        self.motorSpeed = msg.esc

    def calibration_callback(self, msg):
        # Optional: process calibration messages if desired.
        self.get_logger().info("Received calibration message (not used for duty limits in this version)")

    def process_commands(self):
        while True:
            self.control_rudder(self.targetRudderPos)
            self.control_sail(self.targetSailPos)
            self.control_esc(self.motorSpeed)
            time.sleep(0.03)  # Small delay to reduce CPU usage

    def control_rudder(self, value):
        # Smoothly approach the target rudder angle (range: -90 to 90)
        if abs(self.currentRudderPosition - value) > 5:
            if self.currentRudderPosition < value:
                self.currentRudderPosition += 1
            else:
                self.currentRudderPosition -= 1
        duty_cycle = self.map_rudder_duty(self.currentRudderPosition)
        self.rudder_pwm.set_servo_pulsewidth(self.RUDDER_PIN, duty_cycle)

    def control_sail(self, value):
        # Smoothly approach the target sail angle (range: 0 to 180)
        if abs(self.currentSailPosition - value) > 5:
            if self.currentSailPosition < value:
                self.currentSailPosition += 1
            else:
                self.currentSailPosition -= 1
        duty_cycle = self.map_sail_duty(self.currentSailPosition)
        self.sail_pwm.set_servo_pulsewidth(self.SAIL_PIN, duty_cycle)

        sail_msg = Float32()
        sail_msg.data = float(self.currentSailPosition)
        self.currentSailPublisher.publish(sail_msg)

    def control_esc(self, value):
        # For ESC, we use a simple linear mapping (range: -100 to 100)
        duty_cycle = self.map_to_duty_cycle(
            value, 
            min_value=-100, 
            max_value=100, 
            min_duty=500, 
            max_duty=2500
        )
        self.esc_pwm.set_servo_pulsewidth(self.ESC_PIN, duty_cycle)

    def map_rudder_duty(self, angle):
        """
        Map rudder angle (in degrees, range: -90 to 90) to a PWM duty cycle.
        Uses piecewise linear mapping:
          -90° -> minDutyRudder
           0°  -> midDutyRudder
          90°  -> maxDutyRudder
        """
        angle = -angle
        if angle < 0:
            # Map from -90 to 0
            duty = self.minDutyRudder + ((angle + 90) / 90) * (self.midDutyRudder - self.minDutyRudder)
        else:
            # Map from 0 to 90
            duty = self.midDutyRudder + (angle / 90) * (self.maxDutyRudder - self.midDutyRudder)
        return round(duty, 2)

    def map_sail_duty(self, angle):
        """
        Map sail angle (in degrees, range: 0 to 180) to a PWM duty cycle.
        Uses piecewise linear mapping:
          0°   -> minDutySail
          90°  -> midDutySail
          180° -> maxDutySail
        """
        if angle < 90:
            duty = self.minDutySail + (angle / 90) * (self.midDutySail - self.minDutySail)
        else:
            duty = self.midDutySail + ((angle - 90) / 90) * (self.maxDutySail - self.midDutySail)
        return round(duty, 2)

    def map_to_duty_cycle(self, value, min_value, max_value, min_duty, max_duty):
        # A simple linear mapping if needed elsewhere.
        value = max(min_value, min(max_value, value))
        duty_cycle = min_duty + (value - min_value) * (max_duty - min_duty) / (max_value - min_value)
        return round(duty_cycle, 2)

    def destroy_node(self):
        self.rudder_pwm.stop()
        self.sail_pwm.stop()
        self.esc_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BoatControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down BoatControlNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

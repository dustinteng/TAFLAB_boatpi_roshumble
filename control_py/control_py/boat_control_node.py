import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData, CalibrationData  # Add CalibrationData message for calibration inputs
import RPi.GPIO as GPIO
import time
from queue import Queue
import threading
import json
import os

class BoatControlNode(Node):
    def __init__(self):
        super().__init__('boat_control_node')

        # GPIO setup for each servo
        self.RUDDER_PIN = 23
        self.SAIL_PIN = 24
        self.ESC_PIN = 25
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        # Set up GPIO outputs for each servo
        GPIO.setup(self.RUDDER_PIN, GPIO.OUT)
        GPIO.setup(self.SAIL_PIN, GPIO.OUT)
        GPIO.setup(self.ESC_PIN, GPIO.OUT)
        
        # PWM setup
        self.rudder_pwm = GPIO.PWM(self.RUDDER_PIN, 50)
        self.sail_pwm = GPIO.PWM(self.SAIL_PIN, 50)
        self.esc_pwm = GPIO.PWM(self.ESC_PIN, 50)
        self.rudder_pwm.start(0)
        self.sail_pwm.start(0)
        self.esc_pwm.start(0)
        
        # Queue for servo commands
        self.command_queue = Queue()
        self.worker_thread = threading.Thread(target=self.process_commands)
        self.worker_thread.daemon = True
        self.worker_thread.start()

        # Load calibration data
        self.calibration_data = self.load_calibration_data()

        # Subscriber to listen to control commands
        self.subscription = self.create_subscription(
            ControlData,
            '/boatcontrol',
            self.control_callback,
            10
        )
        
        # Subscriber for calibration data
        self.calibration_subscription = self.create_subscription(
            CalibrationData,
            '/calibration',
            self.calibration_callback,
            10
        )

        self.get_logger().info("BoatControlNode initialized and listening to /boatcontrol and /calibration")

    def control_callback(self, msg):
        self.command_queue.put(('rudder', msg.servo_rudder))
        self.command_queue.put(('sail', msg.servo_sail))
        self.command_queue.put(('esc', msg.esc))

    def calibration_callback(self, msg):
        # Save min and max angles from the calibration message
        self.calibration_data = {
            'rudder_min': msg.rudder_min,
            'rudder_max': msg.rudder_max,
            'sail_min': msg.sail_min,
            'sail_max': msg.sail_max,
            'esc_min': msg.esc_min,
            'esc_max': msg.esc_max
        }
        self.save_calibration_data()
        self.get_logger().info("Calibration data updated and saved")

    def get_calibration_data(self, boat_id):
        # Load calibration data specific to this boat (from file or memory)
        file_path = f"/home/boat/{boat_id}_calibration.json"
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                calibration_data = json.load(f)
            return calibration_data
        else:
            return None


    def process_commands(self):
        while True:
            command_type, value = self.command_queue.get()
            if command_type == 'rudder':
                self.control_rudder(value)
            elif command_type == 'sail':
                self.control_sail(value)
            elif command_type == 'esc':
                self.control_esc(value)
            self.command_queue.task_done()
            time.sleep(0.05)  # Small delay to reduce CPU usage

    def control_rudder(self, value):
        duty_cycle = self.map_to_duty_cycle(
            value, 
            min_value=self.calibration_data['rudder_min'], 
            max_value=self.calibration_data['rudder_max'], 
            min_duty=2, 
            max_duty=12
        )
        self.rudder_pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f"Rudder set to: {value} (Duty Cycle: {duty_cycle})")

    def control_sail(self, value):
        duty_cycle = self.map_to_duty_cycle(
            value, 
            min_value=self.calibration_data['sail_min'], 
            max_value=self.calibration_data['sail_max'], 
            min_duty=2, 
            max_duty=12
        )
        self.sail_pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f"Sail set to: {value} (Duty Cycle: {duty_cycle})")

    def control_esc(self, value):
        duty_cycle = self.map_to_duty_cycle(
            value, 
            min_value=self.calibration_data['esc_min'], 
            max_value=self.calibration_data['esc_max'], 
            min_duty=2, 
            max_duty=12
        )
        self.esc_pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f"ESC set to: {value} (Duty Cycle: {duty_cycle})")

    def map_to_duty_cycle(self, value, min_value, max_value, min_duty, max_duty):
        value = max(min_value, min(max_value, value))
        duty_cycle = min_duty + (value - min_value) * (max_duty - min_duty) / (max_value - min_value)
        return round(duty_cycle, 1)

    def save_calibration_data(self):
        try:
            file_path = '/home/boat/Desktop/src/control_py/control_py/calibration_data.json'
            self.get_logger().info(f"Current working directory: {os.getcwd()}")
            with open('calibration_data.json', 'w') as f:
                json.dump(self.calibration_data, f)
            self.get_logger().info("Calibration data saved successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to save calibration data: {e}")


    def load_calibration_data(self):
        # Define the full path to the calibration file
        file_path = '/home/boat/Desktop/src/calibration_data.json'
        # Debug: Print the file path to confirm where it is looking
        self.get_logger().info(f"Attempting to load calibration data from: {file_path}")
        
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                self.get_logger().info("Calibration data found and loaded.")
                return json.load(f)
        else:
            self.get_logger().warning("Calibration data file not found. Using default values.")
            return {
                'rudder_min': -90,
                'rudder_max': 90,
                'sail_min': -90,
                'sail_max': 90,
                'esc_min': -100,
                'esc_max': 100
            }


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

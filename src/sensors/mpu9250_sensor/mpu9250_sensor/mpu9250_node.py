import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import smbus2
import math
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *
import time

# I2C Addresses
MPU9250_ADDR = 0x68  # MPU9250 I2C address
AK8963_ADDR = 0x0C   # Magnetometer I2C address (inside MPU9250)

class MPU9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_node')

        # Declare and allow dynamic updates to parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mag_offset_x', 17.431640625),
                ('mag_offset_y', -12.8173828125),
                ('mag_offset_z', -25.634765625),
            ],
        )

        self.mag_offset_x = self.get_parameter('mag_offset_x').value
        self.mag_offset_y = self.get_parameter('mag_offset_y').value
        self.mag_offset_z = self.get_parameter('mag_offset_z').value

        # Accelerometer Scale Factors and Bias
        self.accel_scale = [1.00023173, 1.0001651, 0.64440996]
        self.accel_bias = [-0.09224956, -0.02142647, 0.15689505]

        # Gyroscope Bias
        self.gyro_bias = [0.5313873291015625, -2.37396240234375, 1.366424560546875]

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, 'mpu9250/imu', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'mpu9250/mag', 10)

        # Timer at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        # Enable I2C bypass mode
        self.enable_i2c_bypass()

        # Initialize the MPU9250 sensor
        self.mpu = MPU9250(
            address_ak=AK8963_ADDR,
            address_mpu_master=MPU9250_ADDR,
            address_mpu_slave=None,
            bus=1,  # I2C bus
            gfs=GFS_250,  # Gyroscope Full Scale (250dps)
            afs=AFS_2G,   # Accelerometer Full Scale (2G)
            mfs=AK8963_BIT_16,  # Magnetometer resolution (16-bit)
            mode=AK8963_MODE_C100HZ  # Magnetometer sample rate (100Hz)
        )
        self.mpu.configure()

    def enable_i2c_bypass(self):
        try:
            bus = smbus2.SMBus(1)  # Use I2C bus 1
            bus.write_byte_data(MPU9250_ADDR, 0x37, 0x02)  # Set BYPASS_EN bit
            bus.write_byte_data(MPU9250_ADDR, 0x24, 0x22)  # I2C Master Control (Optional)
            self.get_logger().info("I2C bypass mode enabled on MPU9250.")
            bus.close()
        except Exception as e:
            self.get_logger().error(f"Error enabling I2C bypass: {e}")

    def publish_sensor_data(self):
        imu_msg = Imu()
        mag_msg = MagneticField()

        # Read sensor data
        accel = self.mpu.readAccelerometerMaster()
        gyro = self.mpu.readGyroscopeMaster()
        mag = self.mpu.readMagnetometerMaster()

        # Apply calibration to accelerometer data
        ax = (accel[0] - self.accel_bias[0]) * self.accel_scale[0]
        ay = (accel[1] - self.accel_bias[1]) * self.accel_scale[1]
        az = (accel[2] - self.accel_bias[2]) * self.accel_scale[2]

        # Apply calibration to gyroscope data (subtract bias)
        gx = gyro[0] - self.gyro_bias[0]
        gy = gyro[1] - self.gyro_bias[1]
        gz = gyro[2] - self.gyro_bias[2]

        # Apply calibration to magnetometer data
        mx = mag[0] - self.mag_offset_x
        my = mag[1] - self.mag_offset_y
        mz = mag[2] - self.mag_offset_z

        # Convert units
        imu_msg.linear_acceleration.x = ax * 9.81
        imu_msg.linear_acceleration.y = ay * 9.81
        imu_msg.linear_acceleration.z = az * 9.81

        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)

        imu_msg.orientation_covariance[0] = -1  # Mark orientation as unknown

        mag_msg.magnetic_field.x = mx * 0.15 / 1000.0
        mag_msg.magnetic_field.y = my * 0.15 / 1000.0
        mag_msg.magnetic_field.z = mz * 0.15 / 1000.0

        mag_msg.magnetic_field_covariance[0] = -1  # Mark covariance as unknown

        heading = self.calculate_heading(mx, my)
        self.get_logger().info(f"Heading: {heading:.2f}°")

        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)

    def calculate_heading(self, mx, my):
        heading_rad = math.atan2(mx, my)
        heading_deg = math.degrees(heading_rad)
        heading_deg += 0  # Apply offset
        if heading_deg < 0:
            heading_deg += 360
        return heading_deg

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

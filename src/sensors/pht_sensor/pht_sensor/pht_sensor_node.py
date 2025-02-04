import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32

# I2C address for the BME280 sensor
BME280_ADDRESS = 0x77  # Address for BME280
TEMP_REGISTER = 0xFA  # Register for temperature
HUMIDITY_REGISTER = 0xFD  # Register for humidity
PRESSURE_REGISTER = 0xF7  # Register for pressure

class PHTSensorNode(Node):
    def __init__(self):
        super().__init__('pht_sensor_node')
        self.bus = SMBus(1)  # Raspberry Pi I2C bus
        
        # Publishers for temperature, humidity, and pressure
        self.temp_publisher = self.create_publisher(Float32, 'pht_temperature', 10)
        self.humidity_publisher = self.create_publisher(Float32, 'pht_humidity', 10)
        self.pressure_publisher = self.create_publisher(Float32, 'pht_pressure', 10)
        
        self.timer = self.create_timer(1.0, self.read_sensor)  # Publish every second
    
    def read_sensor(self):
        try:
            # Read temperature
            temp_data = self.bus.read_i2c_block_data(BME280_ADDRESS, TEMP_REGISTER, 3)
            temperature = ((temp_data[0] << 16 | temp_data[1] << 8 | temp_data[2]) >> 4) / 100.0  # Adjust scale as needed
            self.temp_publisher.publish(Float32(data=temperature))
            self.get_logger().info(f"Temperature: {temperature:.2f} °C")
            
            # Read humidity
            humidity_data = self.bus.read_i2c_block_data(BME280_ADDRESS, HUMIDITY_REGISTER, 2)
            humidity = (humidity_data[0] << 8 | humidity_data[1]) / 1024.0  # Adjust scale as needed
            self.humidity_publisher.publish(Float32(data=humidity))
            self.get_logger().info(f"Humidity: {humidity:.2f} %")
            
            # Read pressure
            pressure_data = self.bus.read_i2c_block_data(BME280_ADDRESS, PRESSURE_REGISTER, 3)
            pressure = ((pressure_data[0] << 16 | pressure_data[1] << 8 | pressure_data[2]) >> 4) / 100.0  # Adjust scale as needed
            self.pressure_publisher.publish(Float32(data=pressure))
            self.get_logger().info(f"Pressure: {pressure:.2f} hPa")

        except Exception as e:
            self.get_logger().error(f"Error reading sensor: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PHTSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import smbus2

bus = smbus2.SMBus(1)  # Use I2C bus 1
address_mpu = 0x68     # MPU9250 main I2C address

# Enable bypass mode
bus.write_byte_data(address_mpu, 0x37, 0x02)

# Configure I2C Master to enable communication
bus.write_byte_data(address_mpu, 0x24, 0x22)

print("Bypass mode enabled.")

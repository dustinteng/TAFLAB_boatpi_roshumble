import time
import smbus2
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *

# I2C Addresses
MPU9250_ADDR = 0x68  # MPU9250 I2C address
AK8963_ADDR = 0x0C   # Magnetometer I2C address (inside MPU9250)

# Enable I2C bypass mode
def enable_i2c_bypass():
    try:
        bus = smbus2.SMBus(1)  # Use I2C bus 1
        bus.write_byte_data(MPU9250_ADDR, 0x37, 0x02)  # Set BYPASS_EN bit
        bus.write_byte_data(MPU9250_ADDR, 0x24, 0x22)  # I2C Master Control (Optional)
        print("I2C bypass mode enabled on MPU9250.")
        bus.close()
    except Exception as e:
        print(f"Error enabling I2C bypass: {e}")

# Call this function before initializing MPU9250
enable_i2c_bypass()

# Initialize MPU9250 sensor
mpu = MPU9250(
    address_ak=AK8963_ADDR,
    address_mpu_master=MPU9250_ADDR,
    address_mpu_slave=None,
    bus=1,  # I2C bus
    gfs=GFS_250,  # Gyroscope Full Scale (250dps)
    afs=AFS_2G,   # Accelerometer Full Scale (2G)
    mfs=AK8963_BIT_16,  # Magnetometer resolution (16-bit)
    mode=AK8963_MODE_C100HZ  # Magnetometer sample rate (100Hz)
)

mpu.configure()  # Apply settings

# Read sensor data in a loop
print("Reading MPU9250 data... Press Ctrl+C to stop.")
try:
    while True:
        accel = mpu.readAccelerometerMaster()
        gyro = mpu.readGyroscopeMaster()
        mag = mpu.readMagnetometerMaster()

        print(f"Accel: X={accel[0]:.2f}g, Y={accel[1]:.2f}g, Z={accel[2]:.2f}g")
        print(f"Gyro: X={gyro[0]:.2f}°/s, Y={gyro[1]:.2f}°/s, Z={gyro[2]:.2f}°/s")
        print(f"Mag: X={mag[0]:.2f}µT, Y={mag[1]:.2f}µT, Z={mag[2]:.2f}µT")
        print("-" * 50)

        time.sleep(0.5)  # Delay for readability

except KeyboardInterrupt:
    print("Stopped by user.")
except Exception as e:
    print(f"Error: {e}")

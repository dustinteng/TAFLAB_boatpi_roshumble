from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

try:
    # Initialize MPU9250
    mpu = MPU9250(
        address_ak=0x0C,  # AK8963 magnetometer address (not used here)
        address_mpu_master=0x68,  # MPU9250 address
        address_mpu_slave=None, 
        bus=1,  # I2C bus
        gfs=GFS_250,  # Gyroscope full-scale range (250 dps)
        afs=AFS_2G,   # Accelerometer full-scale range (2g)
        mfs=None,     # Magnetometer scale not configured
        mode=None     # Magnetometer mode not configured
    )

    print("Calibrating accelerometer and gyroscope... Keep the sensor stationary.")
    # Perform full calibration and skip magnetometer setup
    mpu.calibrate(retry=3)  # Calibrates accelerometer and gyroscope
    print("Calibration complete.")

    print("Configuring sensor...")
    mpu.configure()  # Apply the configuration settings
    print("Configuration complete.")

    # Read sensor data
    print("Reading data...")
    print("Acceleration:", mpu.readAccelerometerMaster())
    print("Gyroscope:", mpu.readGyroscopeMaster())

except Exception as e:
    print(f"An error occurred: {e}")

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

try:
    # Initialize MPU9250
    mpu = MPU9250(
    address_ak=0x0C,  # Correct address for AK8963
    address_mpu_master=0x68,  # MPU9250 address
    address_mpu_slave=None, 
    bus=1,  # I2C bus
    gfs=GFS_250, 
    afs=AFS_2G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ
)


    print("Calibrating sensor... This may take a few seconds.")
    mpu.calibrate()  # Optional: calibrate the sensor
    print("Calibration complete.")

    print("Configuring sensor...")
    mpu.configure()  # Apply the settings
    print("Configuration complete.")

    # Read sensor data
    print("Reading data...")
    print("Acceleration:", mpu.readAccelerometerMaster())
    print("Gyroscope:", mpu.readGyroscopeMaster())
    print("Magnetometer:", mpu.readMagnetometerMaster())

except Exception as e:
    print(f"An error occurred: {e}")

import smbus2

bus = smbus2.SMBus(1)
address_ak = 0x0C  # AK8963 address

# Read the WHO_AM_I register
who_am_i_ak8963 = bus.read_byte_data(address_ak, 0x00)
print(f"AK8963 WHO_AM_I register: {hex(who_am_i_ak8963)}")

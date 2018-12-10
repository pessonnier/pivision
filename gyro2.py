import smbus
bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect
power_mgmt_1 = 0x6b
bus.write_byte_data(address, power_mgmt_1, 0)


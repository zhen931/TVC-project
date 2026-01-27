import math
import time

# import smbus only works on Linux
# will include code for it here

# I2C short distance connection between micro-controller to sensor

class IMU:
    def __init__(self):
        # MPU6050 Registers
        self.PWR_MGMT_1 = 0x6B # master switch 
        self.SMPLRT_DIV = 0x19 # speed of sensors sending new data to registers
        self.CONFIG = 0x1A # noise filter
        self.GYRO_CONFIG = 0x1B # how fast rocket turns
        self.INT_ENABLE = 0x38 

        # top half 8-bit number measured by sensor
        self.ACCEL_XOUT_H = 0x3B 
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        
        # I2C Setup
        #self.bus = smbus.SMBus(1) # Bus 1 is standard for RPi
        self.deviceAddress = 0x68 # Default MPU6050 address
        
        self.initSensor()

    def initSensor(self):
        # Wake up the MPU6050 (it sleeps by default)
        self.bus.write_byte_data(self.deviceAddress, self.PWR_MGMT_1, 0)
        
        # Set Sample Rate (1Khz / (7+1) = 125Hz)
        self.bus.write_byte_data(self.deviceAddress, self.SMPLRT_DIV, 7)
        
        # Set Config (DLPF - Digital Low Pass Filter)
        self.bus.write_byte_data(self.deviceAddress, self.CONFIG, 0)
        
        # Set Gyro Range (+/- 250 deg/s) -> Scale Factor 131.0
        self.bus.write_byte_data(self.deviceAddress, self.GYRO_CONFIG, 0)
        
        # Set Accel Range (+/- 2g) -> Scale Factor 16384.0
        # Register 0x1C (ACCEL_CONFIG) is implicitly 0 (2g) on reset
        
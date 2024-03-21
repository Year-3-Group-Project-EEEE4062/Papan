import os
import sys
import time
import smbus

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

while True:
    imu.readSensor()
    imu.computeOrientation()
    print ("Mag x: {:.2f} ; Mag y : {:.2f} ; Mag z : {:.2f}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
    
    
    



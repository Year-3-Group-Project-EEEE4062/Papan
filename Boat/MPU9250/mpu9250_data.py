import time
import numpy as np
import csv
import smbus
from imusensor.MPU9250 import MPU9250


class SensorReader:
    def __init__(self, address=0x68, bus_num=1):
        """
        Initialize the SensorReader class with the specified address and bus number.
        """
        self.address = address  # MPU9250 sensor address.
        self.bus_num = bus_num  # I2C bus number.
        self.bus = smbus.SMBus(bus_num)  # Create an I2C bus instance.
        self.imu = MPU9250.MPU9250(self.bus, self.address)  # Initialize the MPU9250 sensor.
        self.imu.begin()  # Begin communication with the sensor.

    def calibrate_gyroscope(self, num_samples=100, sample_interval=0.01):
        """
        Calibrate the gyroscope by averaging the bias error.
        """
        print("Calibrating gyroscope...")
        gyro_bias = [0, 0, 0]

        # Take multiple readings and average the bias error.
        for _ in range(num_samples):
            self.imu.readSensor()
            gyro_bias[0] += self.imu.GyroVals[0]
            gyro_bias[1] += self.imu.GyroVals[1]
            gyro_bias[2] += self.imu.GyroVals[2]
            time.sleep(sample_interval)

        # Compute average bias values.
        gyro_bias = [val / num_samples for val in gyro_bias]
        print("Gyroscope bias:", gyro_bias)
        return gyro_bias

    def calibrate_accelerometer(self, num_samples=100, sample_interval=0.01):
        """
        Calibrate the accelerometer by averaging the bias error.
        """
        print("Calibrating accelerometer...")
        accel_bias = [0, 0, 0]

        # Place the accelerometer in 6 different positions and average the bias error.
        for _ in range(num_samples):
            self.imu.readSensor()
            accel_bias[0] += self.imu.AccelVals[0]
            accel_bias[1] += self.imu.AccelVals[1]
            accel_bias[2] += self.imu.AccelVals[2]
            time.sleep(sample_interval)

        # Compute average bias values.
        accel_bias = [val / num_samples for val in accel_bias]
        print("Accelerometer bias:", accel_bias)
        
        accel_bias[2] = accel_bias[2] + 10
        return accel_bias

    def read_sensor_data(self, gyro_bias=None, accel_bias=None):
        """
        Read sensor data and apply calibration offsets if provided.
        """
        # Read sensor data.
        self.imu.readSensor()
            
        # Compute orientation based on sensor data.
        self.imu.computeOrientation()

        # Apply calibration offsets if provided.
        if gyro_bias:
            self.imu.GyroVals[0] -= gyro_bias[0]
            self.imu.GyroVals[1] -= gyro_bias[1]
            self.imu.GyroVals[2] -= gyro_bias[2]
        if accel_bias:
            self.imu.AccelVals[0] -= accel_bias[0]
            self.imu.AccelVals[1] -= accel_bias[1]
            self.imu.AccelVals[2] -= accel_bias[2]
            
            
# Instantiate the SensorReader class.
sensor_reader = SensorReader()

# Calibrate gyroscope and accelerometer.
gyro_bias = sensor_reader.calibrate_gyroscope()
accel_bias = sensor_reader.calibrate_accelerometer()

size = 1000
t = np.zeros(size)
y = np.zeros(size)
i = 0

init_time = time.time()
while(i < size):
    # Read sensor data continuously.
    sensor_reader.read_sensor_data(gyro_bias, accel_bias)
    
    # Print accelerometer, gyroscope, magnetometer, and orientation values.
    #print("Accel x: {:.2f} ; Accel y: {:.2f} ; Accel z: {:.2f}".format(sensor_reader.imu.AccelVals[0], sensor_reader.imu.AccelVals[1], sensor_reader.imu.AccelVals[2]))
    print("Gyro x: {:.2f} ; Gyro y: {:.2f} ; Gyro z: {:.2f}".format(sensor_reader.imu.GyroVals[0], sensor_reader.imu.GyroVals[1], sensor_reader.imu.GyroVals[2]))
    
    t[i] = time.time() - init_time
    #y[i] = sensor_reader.imu.AccelVals[0]
    y[i] = sensor_reader.imu.GyroVals[2]
    
    i += 1
    time.sleep(0.01)
    
# Transpose y and t arrays to write them in columns.
data = np.vstack((t, y)).T

# Write data to CSV file.
with open("gyro_data.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'GyroVals'])  # Write header.
    writer.writerows(data)
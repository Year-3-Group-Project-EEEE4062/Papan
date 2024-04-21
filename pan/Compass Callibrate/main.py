import smbus
import math
import time
import numpy as np


def kalman_filter(sensor_data):
    global Xt, Xt_update, Xt_prev, Pt, Pt_update, Pt_prev, Kt, R, Q
    
    R = 100
    Q = 1
    
    Xt_update = Xt_prev
    Pt_update = Pt_prev + Q
    
    Kt = Pt_update / (Pt_update + R)
    Xt = Xt_update + (Kt * (sensor_data - Xt_update))
    Pt = (1 - Kt) * Pt_update
    
    Xt_prev = Xt
    Pt_prev = Pt
    
    return Xt

# Example usage.
Xt = 0           # Initial state.
Xt_prev = 0      # Initial previous state.
Pt_prev = 1      # Initial covariance.
sensor_data = 0  # Example sensor data.


# I2C address of the QMC5883 sensor.
QMC5883_ADDR = 0x0D

# Register control constants.
Mode_Standby = 0b00000000
Mode_Continuous = 0b00000001

ODR_10Hz = 0b00000000
ODR_50Hz = 0b00000100
ODR_100Hz = 0b00001000
ODR_200Hz = 0b00001100

RNG_2G = 0b00000000
RNG_8G = 0b00010000

OSR_512 = 0b00000000
OSR_256 = 0b01000000
OSR_128 = 0b10000000
OSR_64 = 0b11000000


class MechaQMC5883:
    """
    Class for interfacing with the QMC5883 magnetometer sensor.193.28666666666666, Offset Y: -168.91,   offset_x=142.72, offset_y=-85.62
    """
    def __init__(self, offset_x=183.426, offset_y=-125.94):
        """
        Initializes the MechaQMC5883 object.
        """
        # Initialize the I2C bus.
        self.bus = smbus.SMBus(1)
        # Set the address of the QMC5883 sensor.
        self.address = QMC5883_ADDR
        
        self.offset_x = offset_x
        self.offset_y = offset_y
    
    def WriteReg(self, Reg, val):
        """
        Writes a value to the specified register of the sensor.
        Args:
            Reg: Register address to write to.
            val: Value to write to the register.
        """
        # Write data to the specified register of the sensor.
        self.bus.write_byte_data(self.address, Reg, val)

    def init(self):
        """
        Initializes the QMC5883 sensor with default settings.
        """
        # Configure the sensor for continuous operation with default settings.
        self.WriteReg(0x0B, 0x01)
        self.setMode(Mode_Continuous, ODR_200Hz, RNG_8G, OSR_512)

    def setMode(self, mode, odr, rng, osr):
        """
        Sets the operating mode of the sensor.
        Args:
            mode: Operating mode (Standby or Continuous).
            odr: Output Data Rate.
            rng: Measurement range.
            osr: Over Sampling Rate.
        """
        # Configure the operating mode of the sensor.
        self.WriteReg(0x09, mode | odr | rng | osr)

    def softReset(self):
        """
        Performs a soft reset of the sensor.
        """
        # Send the soft reset command to the sensor.
        self.WriteReg(0x0A, 0x80)

    def read(self):
        """
        Reads the raw sensor data from the QMC5883 sensor.
        Returns:
            Tuple of x, y, and z raw sensor data.
        """
        # Request data from the sensor.
        self.bus.write_byte(self.address, 0x00)
        # Read 6 bytes of data from the sensor.
        data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
        # Combine two bytes to get each axis value.
        x = data[0] | (data[1] << 8)
        y = data[2] | (data[3] << 8)
        z = data[4] | (data[5] << 8)
        # Handle negative values.
        if x >= 2**15:
            x = -32768 - 32768 + x
        if y >= 2**15:
            y = -32768 - 32768 + y
        if z >= 2**15:
            z = -32768 - 32768 + z
        
        x -= self.offset_x
        y -= self.offset_y
        
        return x, y, z
    
    def calibrate(self, num_samples=300):
        """
        Calibrates the sensor to determine the hard iron offset.
        Args:
            num_samples: Number of samples to use for calibration.
        """
        print("Starting calibration. Rotate the sensor in all directions...")
        
        x_samples = np.zeros(num_samples)
        y_samples = np.zeros(num_samples)

        for i in range(num_samples):
            x_raw, y_raw, _ = self.read()
            x_samples[i] = x_raw
            y_samples[i] = y_raw
            print(str(i) + '/' + str(num_samples))
            time.sleep(0.05)  # Adjust as needed for sampling frequency
        
        self.offset_x = sum(x_samples) / len(x_samples)
        self.offset_y = sum(y_samples) / len(y_samples)
        print(f"Calibration complete. Offset X: {self.offset_x}, Offset Y: {self.offset_y}")
    
    def azimuth(self, a, b):
        """
        Calculates the azimuth angle based on the raw sensor data.
        Args:
            a: Raw sensor data for one axis.
            b: Raw sensor data for another axis.
        Returns:
            Azimuth angle in degrees.
        """
        # Calculate the azimuth angle using the sensor data.
        azimuth = int(math.atan2(int(a), int(b)) * (180.0 / math.pi))
        # Ensure the angle is within the range [0, 360).
        if azimuth < 0:
            azimuth = 360 + azimuth
        
        azimuth = 360 - (azimuth + 270 ) % 360
        
        return azimuth


if __name__ == "__main__":
    # Create an instance of the MechaQMC5883 class.
    qmc5883 = MechaQMC5883()
    
    # Initialize the sensor.
    qmc5883.init()
    
    # Calibrate the hard iron offset.
    #qmc5883.calibrate()

    # Continuous loop to read and print sensor data.
    while True:
        # Read sensor data from the QMC5883 sensor.
        x, y, z = qmc5883.read()
        
        # Calculate the azimuth angle based on the sensor data.
        a = qmc5883.azimuth(y, x)
        
        filtered_data = int(kalman_filter(a))
        
        # Print sensor data and azimuth angle.
        print(f"X: {x}, Y: {y}, Z: {z}, Azimuth: {a}. KalmanFilter: {filtered_data}")

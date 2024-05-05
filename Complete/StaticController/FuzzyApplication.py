import time
import math
import csv
import serial
import pynmea2
import RPi.GPIO as GPIO
import Controller 
import Compass 
import Localization
import Buzzer


class FuzzyClass:
    def __init__(self, l_motor, r_motor, init_time):
        """
        Initialize the staticClass object.

        Inputs:
            l_motor (object): Left motor object.
            r_motor (object): Right motor object.
            init_time (object): Initial time of program.
        """
        self.init_time = init_time

        # Initialize left and right motors. 
        self.l_motor = l_motor
        self.r_motor = r_motor
        
        # Create an instance of the Buzzer class with pin 17.
        self.buzzer = Buzzer.BuzzerBuzzer(17)
        
        # Initialize Fuzzy Controller for autonomous operation.
        self.boat_controller = Controller.BoatController()
        self.norm_left_motor_speed = self.boat_controller.load_csv("norm_left_motor_speed.csv")
        self.norm_right_motor_speed = self.boat_controller.load_csv("norm_right_motor_speed.csv")
    
        # Initialize compass module.
        self.compass = Compass.MechaQMC5883()
        self.compass.init()

        # To store initial angle.        
        self.init_angle = None

        # To store initial the gps data.
        self.init_time_string = None
        self.init_date_string = None
        self.init_latitude = None
        self.init_longitude = None

        # To store the gps data.
        self.time_string = None
        self.date_string = None
        self.latitude = None
        self.longitude = None
    
    def setInitData(self, time_string, date_string, latitude, longitude):
        """
        Set initial GPS data received from sensors.

        Inputs:
            time_string (str): Time string.
            date_string (str): Date string.
            latitude (float): Latitude value.
            longitude (float): Longitude value.
        """
        # Update the GPS data.
        self.init_time_string = time_string
        self.init_date_string = date_string
        self.init_latitude = latitude
        self.init_longitude = longitude

        # Read sensor data (x, y, z) from the QMC5883 sensor.
        x, y, z = self.compass.read()    
        # Calculate the azimuth angle using the sensor data (y, x).
        self.init_angle = self.compass.azimuth(y, x)

    def setGPSData(self, time_string, date_string, latitude, longitude):
        """
        Set GPS data received from sensors.

        Inputs:
            time_string (str): Time string.
            date_string (str): Date string.
            latitude (float): Latitude value.
            longitude (float): Longitude value.
        """
        # Update the GPS data.
        self.time_string = time_string
        self.date_string = date_string
        self.latitude = latitude
        self.longitude = longitude

    def calc_angle(self, angle1, angle2):
        """
        Calculate the absolute angle between two angles.

        Args:
            angle1 (float): First angle in degrees.
            angle2 (float): Second angle in degrees.

        Returns:
            float: Absolute angle between the two angles in degrees.
        """
        # Calculate the difference between the angles.
        angle_diff = angle2 - angle1
        
        # Adjust for clockwise and anticlockwise directions.
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        return angle_diff

    def convertTo2DPlane(self, lat, lon, reference_lat, reference_lon):
        """
        Converts latitude and longitude coordinates to 2D plane coordinates (x, y)
        with respect to a reference latitude and longitude.

        Parameters:
            lat (float): Latitude of the point to convert.
            lon (float): Longitude of the point to convert.
            reference_lat (float): Reference latitude.
            reference_lon (float): Reference longitude.

        Returns:
            tuple: A tuple containing the x and y coordinates in meters, distance in meters and the bearing in degrees.
        """

        # Earth radius in meters.
        EARTH_RADIUS = 6371000.0

        # Convert degrees to radians.
        reference_lat = math.radians(reference_lat)
        reference_lon = math.radians(reference_lon)
        lat = math.radians(lat)
        lon = math.radians(lon)

        # Calculate differences in coordinates.
        deltaLat = lat - reference_lat
        deltaLon = lon - reference_lon

        # Calculate distance between the two points.
        a = math.sin(deltaLat / 2)**2 + math.cos(reference_lat) * math.cos(lat) * math.sin(deltaLon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = EARTH_RADIUS * c

        # Calculate bearing.
        angle = math.atan2(math.sin(deltaLon) * math.cos(lat), math.cos(reference_lat) * math.sin(lat) - math.sin(reference_lat) * math.cos(lat) * math.cos(deltaLon))
        bearing = math.degrees(angle)

        # Convert distance and bearing to x-y coordinates.
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        return x, y, distance, bearing
    
    def runStatic(self):
        """
        Perform static control operations and record data to a CSV file.

        This method performs static control operations for autonomous navigation and records relevant data
        such as GPS coordinates, motor speeds, and sensor readings to a CSV file for analysis and logging.

        """
        with open('staticController.csv', 'a', newline='') as csvfile:
            fieldnames = ['Date', 'Time', 'Latitude', 'Longitude', 'Initial Latitude', 'Initial Longitude', 'Bearing', 'Angle', 'Diff Angle', 'Left Motor Speed', 'Right Motor Speed']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            x, y, z = self.compass.read()
            angle = self.compass.azimuth(y, x)

            [x_plane, y_plane, distance, bearing] = self.convertTo2DPlane(self.latitude, self.longitude, self.init_latitude, self.init_longitude)
            bearing += 180
            angle_diff = self.boat_controller.calc_angle(angle, bearing)
            left = 0
            right = 0
            print('Latitude =', self.latitude, ', Longitude =', self.longitude)
            # First control.
            if self.latitude is not None and distance > 1.5:
                # Warning sound.
                self.buzzer.play_warning()

                # Forward movement.
                if abs(angle_diff) < 90:
                    print('b')
                    # First part.
                    left, right = self.boat_controller.fuzzy_controller(1.0, 0, angle_diff / 180.0, self.norm_left_motor_speed, self.norm_right_motor_speed)
                    # Write speed to boat.
                    GPIO.output(20, GPIO.LOW)
                    GPIO.output(21, GPIO.HIGH)
                    self.l_motor.ChangeDutyCycle(left*30)
                    self.r_motor.ChangeDutyCycle(right*30)
                # Backward movement.
                else:
                    print('c')
                    left, right = self.boat_controller.fuzzy_controller(1.0, 0,  angle_diff/180.0, self.norm_left_motor_speed, self.norm_right_motor_speed)
                    # Write speed to boat.
                    GPIO.output(20, GPIO.HIGH)
                    GPIO.output(21, GPIO.LOW)
                    self.l_motor.ChangeDutyCycle(right*30)
                    self.r_motor.ChangeDutyCycle(left*30)
            # Nothing happen.
            else:
                # Write speed to boat.
                print('a')
                GPIO.output(20, GPIO.LOW)
                GPIO.output(21, GPIO.HIGH)
                self.l_motor.ChangeDutyCycle(0)
                self.r_motor.ChangeDutyCycle(0)
                
            writer.writerow({'Date': self.date_string, 'Time': self.time_string, 'Latitude': self.latitude, 'Longitude': self.longitude, 'Initial Latitude': self.init_latitude, 'Initial Longitude': self.init_longitude, 'Bearing': bearing, 'Angle': angle, 'Diff Angle' : angle_diff, 'Left Motor Speed' : left, 'Right Motor Speed': right})
            print(f"Distance = {distance:.2f}", f", Bearing = {bearing:.2f}", ", Angle =", angle, f", Diff Angle = {angle_diff:.2f}", f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}") 


    def runDynamic(self, mode, speed):
        """
        Perform dynamic control operations for remote control.

        Parameters:
            mode (int): Forward mode (1) and reverse mode (0).
            speed (int): Maximum speed of left and right motor.
        """
        # Read sensor data (x, y, z) from the QMC5883 sensor.
        x, y, z = self.compass.read()
    
        # Calculate the azimuth angle using the sensor data (y, x).
        angle = self.compass.azimuth(y, x)
        angle_diff = self.calc_angle(self.init_angle, angle)
    
        left, right = self.boat_controller.inner_controller(1, 0, angle_diff/180.0, self.norm_left_motor_speed, self.norm_right_motor_speed)
        
        # Forward mode.
        if mode == 1:
            GPIO.output(20, GPIO.LOW)
            GPIO.output(21, GPIO.HIGH)
        
        # Reverse mode.
        elif mode == 2:
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(21, GPIO.LOW)

        left *= speed  # Maximum speed for left motor.
        right *= speed  # Maximum speed for right motor.
        self.l_motor.ChangeDutyCycle(left)
        self.r_motor.ChangeDutyCycle(right)
    
        print("Init Angle =", self.init_angle, ", Angle =", angle, ", Diff Angle =", angle_diff, f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}")
            
                        
                
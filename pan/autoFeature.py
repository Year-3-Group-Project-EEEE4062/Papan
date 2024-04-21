import math
import time
import RPi.GPIO as GPIO
import Controller 
import Compass
import Localization
import Filter
import Ultrasonic
import csv
from mssgEncoder import encoderMssg
import gc


class autoClass:
    def __init__(self, l_motor, r_motor, send_cb, init_time):
        """
        Initialize the autoClass object.

        Inputs:
            l_motor (object): Left motor object.
            r_motor (object): Right motor object.
            send_cb (function): Callback function for sending data back to the phone.
        """
        self.init_time = init_time
        
        # Initialize left and right motors. 
        self.l_motor = l_motor
        self.r_motor = r_motor

        # Initialize Fuzzy Controller for autonomous operation.
        self.boat_controller = Controller.BoatController()
        self.norm_left_motor_speed = self.boat_controller.load_csv("norm_left_motor_speed.csv")
        self.norm_right_motor_speed = self.boat_controller.load_csv("norm_right_motor_speed.csv")
        
        # Instantiate AutoController class.
        self.auto = Controller.AutoController()
        # Load data from CSV files.
        self.auto_norm_left_motor_speed = self.auto.load_data('auto_norm_left_motor_speed.csv')
        self.auto_norm_right_motor_speed = self.auto.load_data('auto_norm_right_motor_speed.csv')
        
        # Define GPIO pins for trigger and echo.
        self.ultrasonic_left = Ultrasonic.UltrasonicSensor(trigger_pin=8, echo_pin=7)
        self.ultrasonic_front = Ultrasonic.UltrasonicSensor(trigger_pin=9, echo_pin=11)
        self.ultrasonic_right = Ultrasonic.UltrasonicSensor(trigger_pin=5, echo_pin=6)
    
        # Initialize compass module.
        #self.compass = Compass.MechaQMC5883()
        #self.compass.init()
        
        # Initialize Kalman Filter.
        self.kalman = Filter.KalmanFilter()
        
        # Callback function for wanting to send data back to phone.
        self.send_cb = send_cb
        self.encoder = encoderMssg()

        # Keeping track which set waypoint it should go at
        # using index to keep track.
        self.waypointIndex = 0

        # Create an empty list (suppose to be 2D).
        # Where 1st column = latitude and 2nd column = longitude.
        # Each row represents a set of latitude and longitude.
        self.waypoints = []
        
        # Keep track of how many waypoints have been received (Should be same as length of waypoints array).
        self.obtainedWaypoints = 0
        
        # To store the gps data.
        self.time_string = None
        self.date_string = None
        self.latitude = None
        self.longitude = None
        
        encodedMssg = self.encoder.packIntegers(1, [4])
        self.send_cb(encodedMssg)
    
    def sendBoatLocationToMedium(self):
        """
        Send boat location data to the communication medium.
        """
        mssg = [0, self.latitude, self.longitude]

        encodedMssg = self.encoder.packFloats(1, mssg)
        self.send_cb(encodedMssg)
    
    def sendWaypointTracker(self):
        """
        Send boat location data to the communication medium.
        """
        mssg = [6, self.waypointIndex+1]

        encodedMssg = self.encoder.packIntegers(1, mssg)
        self.send_cb(encodedMssg)
    
    def sendAutoModeFinished(self):
        """
        Send boat location data to the communication medium.
        """
        mssg = [5]

        encodedMssg = self.encoder.packIntegers(1, mssg)
        self.send_cb(encodedMssg)
        
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

    def processAutoMssg(self, autoMssg, boatBusy, quitTemperature, takeReading):
        """
        Process auto messages obtained from Medium.

        Inputs:
            autoMssg (list): Unsigned 8-bit integer list (from Medium).
            boatBusy (bool): Boat's current busy status before processing message.
            quitTemperature (bool): Start or stop temperature measurement.
            takeReading (bool): Receiving or not receiving new mssg.

        Returns:
            Boolean: Start or stop temperature measurement, Receiving or not receiving new mssg.
        """

        # Check if user wants boat location or not.
        # This the only auto message that is not affected by boat busy status.
        if (autoMssg[0] == 0):
            # User wants location of the boat
            self.sendBoatLocationToMedium()

        elif (autoMssg[0] == 1):
            if boatBusy == True:
                # Cancel temperature measuring task.
                quitTemperature = True
            
            print("Cancel all waypoints!")
            
            # Reset variables.
            self.waypoints = []
            self.obtainedWaypoints = 0
            self.waypointIndex = 0
            
            # Send reply message to let Medium know that any operation is cancelled.
            # mode = 4 to indicate waypoint received.
            encodedMssg = self.encoder.packIntegers(1, [4])
            self.send_cb(encodedMssg)

        # User taking new waypoints.
        elif (autoMssg[0] == 2):
            # This means instruction contains a set of lat and lng to be stored.
            # Update number of obtained waypoints.
            self.obtainedWaypoints = self.obtainedWaypoints + 1

            # Extract the lat and long.
            tmp = [autoMssg[1], autoMssg[2]]

            # Append to existing waypoint list if new waypoint not in existing one.
            if tmp not in self.waypoints:
                print("Received: ",self.obtainedWaypoints)
                self.waypoints.append(tmp)

            # Send reply message to let Medium know that message was received.
            # mode = 1 to indicate waypoint received.
            encodedMssg = self.encoder.packIntegers(1, [1, self.obtainedWaypoints])
            self.send_cb(encodedMssg)
            
            takeReading = True
        
        # User done taking new waypoints.
        elif (autoMssg[0] == 3):
            # This message indicates that waypoints have been finalized.
            # This message also contains how many waypoints the boat should've obtained.
            expectedWaypoints = autoMssg[1]

            # Check if obtained same as expected waypoints.
            if expectedWaypoints == self.obtainedWaypoints:
                # Boat will start auto operation
                print("Got: ",len(self.waypoints)," waypoints")

                # Send message to user auto operation will start.
                encodedMssg = self.encoder.packIntegers(1, [2])
                self.send_cb(encodedMssg)

            else:
                print("Got: ",len(self.waypoints)," waypoints")
                
                # Send error message back to the phone (Can try remove newly added waypoints).
                # Reset variables.
                self.waypoints = []
                self.obtainedWaypoints = 0
                self.waypointIndex = 0

                encodedMssg = self.encoder.packIntegers(1, [3])
                self.send_cb(encodedMssg)
            
            takeReading = False
        
        return quitTemperature, takeReading
    
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
    
    def calcAngle(self, angle1, angle2):
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

        return -1*angle_diff

    def runAutonomous(self, boatBusy):
        """
        Runs the autonomous mode

       Args:
            boatBusy (Boolean): Boat busy state.

        Returns:
            Boolean: Boat busy state.
        """

        # Check if the current self.waypointIndex is larger than self.waypoint index range
        """
        Example:
            There is 3 waypoints. so index range of self.waypoints should only be 0,1,2
            This means there is no such a thing as the (int 3) in the index range of self.waypoints in this case.
            So if self.waypointIndex === len(self.waypoint), this means boat has gone through all waypoints
        """
        
        print("Waypoint Index = " + str(self.waypointIndex), end=', ')
        
        with open('autoMode.csv', 'a', newline='') as csvfile:
            fieldnames = ['Real Time', 'Date', 'Time', 'Latitude', 'Longitude', 'Waypoint Index', 'Setpoint Latitude', 'Setpoint Longitude', 'Bearing', 'Angle', 'Diff Angle', 'Left Motor Speed', 'Right Motor Speed']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            #writer.writeheader()
            
            # Measure distances from all three sensors.
            distance_left = self.ultrasonic_left.measure_distance()
            distance_front = self.ultrasonic_front.measure_distance()
            distance_right = self.ultrasonic_right.measure_distance()
            
            # Print distances.
            print("Left Distance: {:.1f} cm".format(distance_left))
            print("Front Distance: {:.1f} cm".format(distance_front))
            print("Right Distance: {:.1f} cm".format(distance_right))
            print("--------------------------")
            flag = (distance_left<150) or (distance_front<150) or (distance_right<150)
            if flag:
                ########################################################################
                # Auton send boat location to phone in every 2.5s.
                if (time.time()-self.init_time)%2.5 > 2:
                    print("Auto send GPS location to boat.")
                    self.sendBoatLocationToMedium()
                    
                left, right = self.auto.auto_controller(distance_left/150.0, distance_front/150.0, distance_right/150.0, self.auto_norm_left_motor_speed, self.auto_norm_right_motor_speed)
                
                left *= 10
                right *= 10
                
                if left > 0:
                    GPIO.output(20, GPIO.HIGH) # Rotate clockwise.
                else:
                    GPIO.output(20, GPIO.LOW) # Rotate anticlockwise.
                    
                if right > 0:
                    GPIO.output(21, GPIO.LOW) # Rotate clockwise.
                else:
                    GPIO.output(21, GPIO.HIGH) # Rotate anticlockwise.
        
                GPIO.output(20, GPIO.HIGH)
                GPIO.output(21, GPIO.LOW)
                self.l_motor.ChangeDutyCycle(abs(left))
                self.r_motor.ChangeDutyCycle(abs(right))

                print(f"Left Distance = {distance_left:.2f}", f", Centre Distance = {distance_front:.2f}", f", Right Distance = {distance_right:.2f}", f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}")
                writer.writerow({'Real Time': time.time()-self.init_time, 'Date': self.date_string, 'Time': self.time_string, 'Latitude': self.latitude, 'Longitude': self.longitude, 'Waypoint Index':self.waypointIndex, 'Setpoint Latitude': 0, 'Setpoint Longitude': 0, 'Bearing': 0, 'Angle': 0, 'Diff Angle' : 0, 'Left Motor Speed' : left, 'Right Motor Speed': right})
                
            elif self.waypointIndex < self.obtainedWaypoints:
                ########################################################################
                # Auton send boat location to phone in every 2.5s.
                if (time.time()-self.init_time)%2.5 > 2:
                    print("Auto send GPS location to boat.")
                    self.sendBoatLocationToMedium()
                    
                # Get the waypoint based on waypoint index.
                current_waypoint = self.waypoints[self.waypointIndex]

                # Extract exact lat and lng of current set waypoint.
                setpoint_latitude = current_waypoint[0]
                setpoint_longitude = current_waypoint[1]

                # Get current compass reading.
                #x, y, z = self.compass.read()
                #angle = self.compass.azimuth(y, x)
                #filtered_angle = int(self.kalman.update(angle))
                #print(angle, filtered_angle)
                angle = 90
                
                # Localize the current with the set lat and lng.
                x_plane, y_plane, distance, bearing = self.convertTo2DPlane(self.latitude, self.longitude, setpoint_latitude, setpoint_longitude)
                bearing += 180
                angle_diff = self.calcAngle(angle, bearing)
                
                left, right = self.boat_controller.fuzzy_controller(distance/5.0, 0, angle_diff/180.0, self.norm_left_motor_speed, self.norm_right_motor_speed)
                left *= 10
                right *= 10

                GPIO.output(20, GPIO.HIGH)
                GPIO.output(21, GPIO.LOW)
                self.l_motor.ChangeDutyCycle(left)
                self.r_motor.ChangeDutyCycle(right)
                
                # Check if boat has reached a radius of 2 meters from the set waypoint.
                if distance < 2.5:
                    # This indicate boat has reach set waypoint.
                    # Stop the boat.
                    self.l_motor.ChangeDutyCycle(0)
                    self.r_motor.ChangeDutyCycle(0)
                    
                    # let user know waypoint reached 
                    self.sendWaypointTracker();
                    
                    # update waypoint index
                    self.waypointIndex = self.waypointIndex + 1
                    
                    # Start temp operation
                    boatBusy = True
 
                print(f"Distance = {distance:.2f}", f", Bearing = {bearing:.2f}", ", Angle =", angle, f", Diff Angle = {angle_diff:.2f}", f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}")
                writer.writerow({'Real Time': time.time()-self.init_time, 'Date': self.date_string, 'Time': self.time_string, 'Latitude': self.latitude, 'Longitude': self.longitude, 'Waypoint Index':self.waypointIndex, 'Setpoint Latitude': setpoint_latitude, 'Setpoint Longitude': setpoint_longitude, 'Bearing': bearing, 'Angle': angle, 'Diff Angle' : angle_diff, 'Left Motor Speed' : left, 'Right Motor Speed': right})
            
            elif self.waypoints == []:
                print('No waypoints in array.')
                
                # Stop the boat.
                self.l_motor.ChangeDutyCycle(0)
                self.r_motor.ChangeDutyCycle(0)
                
                # Set boat status back to free.
                bostBusy = None

            else:
                # Stop the boat.
                self.l_motor.ChangeDutyCycle(0)
                self.r_motor.ChangeDutyCycle(0)
                
                # Reset variables.
                self.waypoints = []
                self.obtainedWaypoints = 0
                self.waypointIndex = 0
                
                # Set boat status back to free.
                bostBusy = None

                # To let user know autonomous operation finished.
                self.sendAutoModeFinished()
                print("Autonomous operation completed!")

        return boatBusy

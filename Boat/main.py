import time
import math
import csv
import serial
import pynmea2
import RPi.GPIO as GPIO
import Controller 
import Compass 


DELAY_TIME = 0.1        # Delay time in seconds.


# Initial and setpoint for GPS (Centre of the bridge).
#setpoint_latitude = 2.944669
#setpoint_longitude = 101.874265

# Setpoint for swimming pool.
#setpoint_latitude = 2.943546
#setpoint_longitude = 101.877711

# Second point
setpoint_latitude = 2.943748
setpoint_longitude = 101.877669


def parse_latitude(data):
    """
    Parse latitude from the NMEA data.

    Returns:
        float: Latitude in decimal degrees.
    """
    lat_data = data.split(',')[3:5]
    latitude = float(lat_data[0][0:2]) + float(lat_data[0][2:]) / 60
    if lat_data[1] == 'S':
        latitude = -latitude
    return latitude


def parse_longitude(data):
    """
    Parse longitude from the NMEA data.

    Returns:
        float: Longitude in decimal degrees.
    """
    lon_data = data.split(',')[5:7]
    longitude = float(lon_data[0][0:3]) + float(lon_data[0][3:]) / 60
    if lon_data[1] == 'W':
        longitude = -longitude
    return longitude


def parse_time(data):
    """
    Parse time from the NMEA data.

    Returns:
        tuple: Hour, minute, and second.
    """
    time_data = data.split(',')[1]
    hour = int(time_data[0:2])
    minute = int(time_data[2:4])
    second = int(time_data[4:6])
    return hour, minute, second


def parse_date(data):
    """
    Parse date from the NMEA data.

    Returns:
        tuple: Day, month, and year.
    """
    date_data = data.split(',')[9]
    day = int(date_data[0:2])
    month = int(date_data[2:4])
    year = int(date_data[4:6])
    return day, month, year


def initialize_motors(enA=12, in1=20, enB=13, in2=21):
    """
    Initialize GPIO pins and motors.

    Returns:
        tuple: Left motor PWM object, Right motor PWM object.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup left motor.
    GPIO.setup(enA, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.output(in1, GPIO.HIGH) # Rotate clockwise.
    l = GPIO.PWM(enA, 1000)     # PWM signal on pin enA with frequency of 1000Hz.
    l.start(0)

    # Setup right motor.
    GPIO.setup(enB, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.output(in2, GPIO.LOW)  # Rotate anticlockwise.
    r = GPIO.PWM(enB, 1000)     # PWM signal on pin enB with frequency of 1000Hz.
    r.start(0)

    return l, r


def stop_movement(left, right, enA=12, in1=20, enB=13, in2=21):
    """
    Function to stop the movement of the motors.

    Args:
        left: Left motor PWM object.
        right: Right motor PWM object.
        enA (int): GPIO pin for left motor enable.
        in1 (int): GPIO pin for left motor input 1.
        enB (int): GPIO pin for right motor enable.
        in2 (int): GPIO pin for right motor input 2.

    Returns:
        None.
    """
    # Set stop speed.
    stop_speed = 20
    
    # Set motor directions to stop.
    GPIO.output(in1, GPIO.LOW)    # Rotate anticlockwise.
    GPIO.output(in2, GPIO.HIGH)   # Rotate clockwise.
    
    # Apply stop speed to motors.
    left.ChangeDutyCycle(stop_speed)
    right.ChangeDutyCycle(stop_speed)

    # Delay for stabilization.
    time.sleep(1)

    # Set motor directions back to normal before stop.
    GPIO.output(in1, GPIO.HIGH)    # Rotate anticlockwise.
    GPIO.output(in2, GPIO.LOW)     # Rotate clockwise.
    
    # Turn off motors.
    left.ChangeDutyCycle(0)
    right.ChangeDutyCycle(0)


def calc_angle(angle1, angle2):
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


def convertTo2DPlane(lat, lon, reference_lat, reference_lon):
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
    
    
def main():
    try:
        with open('main.csv', 'w', newline='') as csvfile:
            fieldnames = ['Date', 'Time', 'Latitude', 'Longitude', 'Bearing', 'Angle', 'Diff Angle', 'Left Motor Speed', 'Right Motor Speed']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            l_motor, r_motor = initialize_motors()
            
            boat_controller = Controller.BoatController()
            norm_left_motor_speed = boat_controller.load_csv("norm_left_motor_speed.csv")
            norm_right_motor_speed = boat_controller.load_csv("norm_right_motor_speed.csv")
            
            compass = Compass.MechaQMC5883()
            compass.init()

            latitude = None
            longitude = None
            date_string = None
            time_string = None
            port = "/dev/ttyS0"
            flag = True
            while True:
                ser = serial.Serial(port, baudrate=9600, timeout=0.5)
                dataout = pynmea2.NMEAStreamReader()
                nmea_sentence_bytes = ser.readline()
                nmea_sentence_bytes = nmea_sentence_bytes.decode('utf-8')

                if '$GNRMC' in nmea_sentence_bytes:
                    try:
                        latitude = round(parse_latitude(nmea_sentence_bytes), 6)
                        longitude = round(parse_longitude(nmea_sentence_bytes), 6)
                        day, month, year = parse_date(nmea_sentence_bytes)
                        date_string = "{}/{}/{}".format(day, month, year)
                        hour, minute, second = parse_time(nmea_sentence_bytes)
                        time_string = "{:02d}:{:02d}:{:02d}".format(hour, minute, second)
                        
                    except ValueError as e:
                        print("Error:", e)
                
                
                if not(latitude == None and flag):
                    x, y, z = compass.read()
                    angle = compass.azimuth(y, x)

                    x_plane, y_plane, distance, bearing = convertTo2DPlane(latitude, longitude, setpoint_latitude, setpoint_longitude)
                    bearing += 180
                    angle_diff = calc_angle(angle, bearing)
                    
                    left, right = boat_controller.inner_controller(distance / 5.0, 0, angle_diff / 180.0, norm_left_motor_speed, norm_right_motor_speed)
                    left *= 20
                    right *= 20

                    GPIO.output(20, GPIO.HIGH)
                    GPIO.output(21, GPIO.LOW)
                    l_motor.ChangeDutyCycle(left)
                    r_motor.ChangeDutyCycle(right)
                    
                    if distance < 2:
                        flag = False
                        stop_movement(l_motor, r_motor)

                    writer.writerow({'Date': date_string, 'Time': time_string, 'Latitude': latitude, 'Longitude': longitude, 'Bearing': bearing, 'Angle': angle, 'Diff Angle' : angle_diff, 'Left Motor Speed' : left, 'Right Motor Speed': right})
                    
                    print(f"Distance = {distance:.2f}", f", Bearing = {bearing:.2f}", ", Angle =", angle, f", Diff Angle = {angle_diff:.2f}", f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}")
                    #print(f"Bearing = {bearing:.2f}", f"Latitude = {latitude:.6f}", f"Longitude = {longitude:.6f}")
                    

    except KeyboardInterrupt:
        stop_movement(l_motor, r_motor)


main()
import time
import csv
import serial
import pynmea2
import RPi.GPIO as GPIO
import Controller 
import Compass 
import Localization


# Initial and setpoint for GPS (Centre of the bridge).
#setpoint_latitude = 2.944669
#setpoint_longitude = 101.874265

# Setpoint for swimming pool.
#setpoint_latitude = 2.943546
#setpoint_longitude = 101.877711

# Second point
setpoint_latitude = 2.943748
setpoint_longitude = 101.877669


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


def main():
    try:
        with open('boat.csv', 'w', newline='') as csvfile:
            fieldnames = ['Date', 'Time', 'Latitude', 'Longitude', 'Bearing', 'Angle', 'Diff Angle', 'Left Motor Speed', 'Right Motor Speed']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            # Initialization for both motor.
            l_motor, r_motor = initialize_motors()
            
            # Initialization for controller.
            boat_controller = Controller.BoatController()
            norm_left_motor_speed = boat_controller.load_csv("norm_left_motor_speed.csv")
            norm_right_motor_speed = boat_controller.load_csv("norm_right_motor_speed.csv")
            
            # Initialization for compass.
            compass = Compass.MechaQMC5883()
            compass.init()
            
            # Initialization for localization.
            gps = Localization.GPS()
            
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
                        latitude = round(gps.parse_latitude(nmea_sentence_bytes), 6)
                        longitude = round(gps.parse_longitude(nmea_sentence_bytes), 6)
                        day, month, year = gps.parse_date(nmea_sentence_bytes)
                        date_string = "{}/{}/{}".format(day, month, year)
                        hour, minute, second = gps.parse_time(nmea_sentence_bytes)
                        time_string = "{:02d}:{:02d}:{:02d}".format(hour, minute, second)
                        
                    except ValueError as e:
                        print("Error:", e)
                
                #latitude = 2.943546
                #longitude = 101.877711
                if not(latitude == None):
                    x, y, z = compass.read()
                    angle = compass.azimuth(y, x)

                    [x_plane, y_plane, distance, bearing] = gps.convertTo2DPlane(latitude, longitude, setpoint_latitude, setpoint_longitude)
                    bearing += 180
                    angle_diff = boat_controller.calc_angle(angle, bearing)
                    
                    left, right = boat_controller.fuzzy_controller(distance / 5.0, 0, angle_diff / 180.0, norm_left_motor_speed, norm_right_motor_speed)
                    
                    # Check condition to stop boat or not.
                    if distance > 2:
                        left *= 20
                        right *= 20
                    else:
                        left = 0
                        right = 0
                    
                    # Write speed to boat.
                    GPIO.output(20, GPIO.HIGH)
                    GPIO.output(21, GPIO.LOW)
                    l_motor.ChangeDutyCycle(left)
                    r_motor.ChangeDutyCycle(right)
                    
                    writer.writerow({'Date': date_string, 'Time': time_string, 'Latitude': latitude, 'Longitude': longitude, 'Bearing': bearing, 'Angle': angle, 'Diff Angle' : angle_diff, 'Left Motor Speed' : left, 'Right Motor Speed': right})
                    print(f"Distance = {distance:.2f}", f", Bearing = {bearing:.2f}", ", Angle =", angle, f", Diff Angle = {angle_diff:.2f}", f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}")
                else:
                    print("Please wait, no GPS reading.")
                    
    except KeyboardInterrupt:
        print("Interrupt boat stop.")


main()
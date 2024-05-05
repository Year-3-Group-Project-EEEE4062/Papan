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
    GPIO.output(in1, GPIO.LOW) # Rotate clockwise.
    l = GPIO.PWM(enA, 1000)     # PWM signal on pin enA with frequency of 1000Hz.
    l.start(0)

    # Setup right motor.
    GPIO.setup(enB, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.output(in2, GPIO.HIGH)  # Rotate anticlockwise.
    r = GPIO.PWM(enB, 1000)     # PWM signal on pin enB with frequency of 1000Hz.
    r.start(0)

    return l, r


def main():
    try:
        with open('staticController.csv', 'w', newline='') as csvfile:
            fieldnames = ['Date', 'Time', 'Latitude', 'Longitude', 'Initial Latitude', 'Initial Longitude', 'Bearing', 'Angle', 'Diff Angle', 'Left Motor Speed', 'Right Motor Speed']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            # Create an instance of the Buzzer class with pin 18.
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            buzzer = Buzzer.BuzzerBuzzer(22)

            # Initialization for both motor.
            l_motor, r_motor = initialize_motors()
            
            # Initialization for controller.
            boat_controller = Controller.BoatController()
            norm_left_motor_speed = boat_controller.load_csv("norm_left_motor_speed.csv")
            norm_right_motor_speed = boat_controller.load_csv("norm_right_motor_speed.csv")
            
            # Initialization for compass.
            compass = Compass.MechaQMC5883()
            compass.init()
            
            # Read sensor data (x, y, z) from the QMC5883 sensor.
            x, y, z = compass.read()
            
            # Calculate the azimuth angle using the sensor data (y, x).
            init_angle = compass.azimuth(y, x)
        
            # Initialization for localization.
            gps = Localization.GPS()
            
            init_latitude = None
            init_longitude = None
            init_date_string = None
            init_time_string = None
            
            latitude = None
            longitude = None
            date_string = None
            time_string = None
            tmp_latitude = None
            tmp_longitude = None
            print("Attempting to get GPS reading..")
            # Try until get first gps reading.
            # Once gotten 1st gps reading, start the main while loop.
            flag=1
            while True:
                try:
                    [init_latitude, init_longitude, init_date_string, init_time_string] = gps.read_serial_gps()
                    
                    init_latitude =  2.944611
                    init_longitude = 101.874219
                    init_date_string = '28/3/2024'
                    init_time_string = '11:23'
            
                    if init_latitude != None and init_longitude != None:
                        print('1st GPS reading.')
                        print('Time = ' + init_time_string+', Date = ' + init_date_string + ', Latitude = ' + str(init_latitude) + ', Longitude = ' + str(init_longitude))
                        latitude = init_latitude
                        longitude = init_longitude
                        date_string = init_date_string
                        time_string = init_time_string
                        
                        break
                    
                except:
                    print('a')
                    pass
            

            #print("Obtained GPS reading..")
            print("Program started..")
    
            port = "/dev/ttyAMA0"
            
            while True:
                    # Get the GPS data for both remote and auto instances.
                try:
                    [tmp_latitude, tmp_longitude, tmp_date_string, tmp_time_string] = gps.read_serial_gps()
                    
                    # Write GPS data to remote and autonomous array.
                    if tmp_latitude != None and tmp_longitude != None:
                        latitude = tmp_latitude
                        longitude = tmp_longitude
                        date_string = tmp_date_string
                        time_string = tmp_time_string
                except:
                    pass
                
                x, y, z = compass.read()
                angle = compass.azimuth(y, x)

                [x_plane, y_plane, distance, bearing] = gps.convertTo2DPlane(latitude, longitude, init_latitude, init_longitude)
                bearing += 180
                angle_diff = boat_controller.calc_angle(angle, bearing)
                left = 0
                right = 0
                print('Latitude =', latitude, ', Longitude =', longitude)
                # First control.
                if latitude != None and distance > 1.5:
                    # Warning sound.
                    buzzer.play_warning()

                    if abs(angle_diff) < 90:
                        print('b')
                        # First part.
                        left, right = boat_controller.fuzzy_controller(1.0, 0, angle_diff / 180.0, norm_left_motor_speed, norm_right_motor_speed)
                        # Write speed to boat.
                        GPIO.output(20, GPIO.LOW)
                        GPIO.output(21, GPIO.HIGH)
                        l_motor.ChangeDutyCycle(left*25)
                        r_motor.ChangeDutyCycle(right*25)
                    # Second control.
                    else:
                        print('c')
                        left, right = boat_controller.fuzzy_controller(1.0, 0,  angle_diff/180.0, norm_left_motor_speed, norm_right_motor_speed)
                        # Write speed to boat.
                        GPIO.output(20, GPIO.HIGH)
                        GPIO.output(21, GPIO.LOW)
                        l_motor.ChangeDutyCycle(right*25)
                        r_motor.ChangeDutyCycle(left*25)
                # Nothing happen.
                else:
                    # Write speed to boat.
                    print('a')
                    GPIO.output(20, GPIO.LOW)
                    GPIO.output(21, GPIO.HIGH)
                    l_motor.ChangeDutyCycle(0)
                    r_motor.ChangeDutyCycle(0)
                    
                writer.writerow({'Date': date_string, 'Time': time_string, 'Latitude': latitude, 'Longitude': longitude, 'Initial Latitude': init_latitude, 'Initial Longitude': init_longitude, 'Bearing': bearing, 'Angle': angle, 'Diff Angle' : angle_diff, 'Left Motor Speed' : left, 'Right Motor Speed': right})
                print(f"Distance = {distance:.2f}", f", Bearing = {bearing:.2f}", ", Angle =", angle, f", Diff Angle = {angle_diff:.2f}", f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}")    
    
    except KeyboardInterrupt:
        print("Interrupt boat stop.")
        
        # Write speed to boat.
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.HIGH)
        l_motor.ChangeDutyCycle(0)
        r_motor.ChangeDutyCycle(0)


main()
import time
import csv
import Controller
import RPi.GPIO as GPIO
import Compass 


# Function to initialize GPIO pins and motors.
def initialize_motors():
    enA = 12   # Pin for left motor.
    in1 = 20
    enB = 13   # Pin for right motor.
    in2 = 21

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup left motor.
    GPIO.setup(enA, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.output(in1, GPIO.HIGH)  # Rotate clockwise.
    l = GPIO.PWM(enA, 1000)  # PWM signal on pin enA with frequency of 1000Hz.
    l.start(0)

    # Setup right motor.
    GPIO.setup(enB, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.output(in2, GPIO.LOW)  # Rotate anticlockwise.
    r = GPIO.PWM(enB, 1000)  # PWM signal on pin enB with frequency of 1000Hz.
    r.start(0)

    return l, r


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

    return angle_diff


# Main function.
def main():
    # Open the CSV file in 'w' mode to create a new file.
    with open('Fuzzy.csv', 'w', newline='') as csvfile:
        # Define the fieldnames for the CSV file.
        fieldnames = ['Time', 'Diff Angle', 'Left Motor Speed', 'Right Motor Speed']

        # Create a CSV writer object.
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write the header row to the CSV file.
        writer.writeheader()
        
        l_motor, r_motor = initialize_motors()  # Initialize motors.
        boat_controller = Controller.BoatController()  # Create controller instance.

        # Load normalized motor speed data from CSV files.
        norm_left_motor_speed = boat_controller.load_csv("norm_left_motor_speed.csv")
        norm_right_motor_speed = boat_controller.load_csv("norm_right_motor_speed.csv")

        # Initialize compass sensor.
        compass = Compass.MechaQMC5883()

        # Initialize the sensor and start continuous reading of sensor data.
        compass.init()
        
        # Read sensor data (x, y, z) from the QMC5883 sensor.
        x, y, z = compass.read()
        
        # Calculate the azimuth angle using the sensor data (y, x).
        init_angle = compass.azimuth(y, x)
        
        # Initial time.
        init_time = time.time()
        
        try:
            # Continuous loop for controlling the boat.
            while True:
                # Read sensor data (x, y, z) from the QMC5883 sensor.
                x, y, z = compass.read()
            
                # Calculate the azimuth angle using the sensor data (y, x).
                angle = compass.azimuth(y, x)
            
                angle_diff = calc_angle(init_angle, angle)
            
                left, right = boat_controller.inner_controller(1, 0, angle_diff/180.0, norm_left_motor_speed, norm_right_motor_speed)
      
                left *= 20  # Maximum speed for left motor.
                right *= 20  # Maximum speed for right motor.
                
                l_motor.ChangeDutyCycle(left)
                r_motor.ChangeDutyCycle(right)
                
                # Write data to CSV file.
                writer.writerow({'Time': time.time()-init_time, 'Diff Angle': angle_diff, 'Left Motor Speed':left, 'Right Motor Speed':right})
                
                print("Init Angle =", init_angle, ", Angle =", angle, ", Diff Angle =", angle_diff, f", Left Motor = {left:.2f}", f", Right Motor = {right:.2f}")
        
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting program.")
            l_motor.ChangeDutyCycle(0)
            r_motor.ChangeDutyCycle(0)
            # CSV file is automatically closed when exiting the 'with' block.


if __name__ == "__main__":
    main()  # Call main function.

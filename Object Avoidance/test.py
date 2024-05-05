import RPi.GPIO as GPIO
import time
import Controller
from Localization import GPS


def initializeMotors(enA=12, in1=20, enB=13, in2=21):
    """
    Initialize GPIO pins and motors.

    Args:
        enA (int): GPIO pin number for enabling Left motor.
        in1 (int): GPIO pin number for controlling Left motor direction.
        enB (int): GPIO pin number for enabling Right motor.
        in2 (int): GPIO pin number for controlling Right motor direction.

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


class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin):
        """
        Initialize the UltrasonicSensor object with trigger and echo pins.
        
        Args:
            trigger_pin (int): GPIO pin number for trigger.
            echo_pin (int): GPIO pin number for echo.
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.setup_ultrasonic()

    def setup_ultrasonic(self):
        """
        Setup the GPIO pins for ultrasonic sensor.
        """
        # Set GPIO mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # Set up GPIO pins
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def measure_distance(self):
        """
        Measure the distance using ultrasonic sensor.
        
        Returns:
            float: Distance measured in centimeters.
        """
        # Send 10us pulse to trigger pin.
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)
        
        # Measure echo pulse duration.
        start_time = time.time()
        stop_time = start_time
    
        # Wait for the echo pulse to start.
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()
            # Timeout if no echo received within 0.1 second.
            if start_time - stop_time > 0.1:
                return 150  # Return 150 to indicate timeout.

        # Wait for the echo pulse to end.
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()
            # Timeout if echo pulse lasts longer than 0.1 second.
            if stop_time - start_time > 0.1:
                return 150  # Return 150 to indicate timeout.

        # Calculate distance.
        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound is 34300 cm/s.
        
        if distance > 150:
            return 150
        
        return distance



if __name__ == "__main__":
    # Create instance of GPS class for localization.
    localization = GPS()

    # GPS data.
    latitude  = None
    longitude = None
    date_string = None
    time_string = None

    print("Attempting to get GPS reading..")
    # Try until get first gps reading.
    # Once gotten 1st gps reading, start the main while loop.
    while True:
        try:
            [latitude, longitude, date_string, time_string] = localization.read_serial_gps()
            
#             latitude =  2.944611
#             longitude = 101.874219
#             date_string = '28/3/2024'
#             time_string = '11:23'
            
            if latitude != None and longitude != None:
                break
            else:
                print("Trying to get 1st GPS reading.")
                continue
        except:
            pass

    print("Obtained GPS reading..")
    print("Program started..")

    # Open the CSV file in 'w' mode to create a new file.
    with open('Avoid.csv', 'w', newline='') as csvfile:
        # Define the fieldnames for the CSV file.
        fieldnames = ['Time', 'Latitude', 'Longitude', 'Left Distance', 'Front Distance', 'Right Distance', 'Left Motor Speed', 'Right Motor Speed']

        # Create a CSV writer object.
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write the header row to the CSV file.
        writer.writeheader()

        # Define GPIO pins for trigger and echo.
        ultrasonic_left = UltrasonicSensor(trigger_pin=8, echo_pin=7)
        ultrasonic_front = UltrasonicSensor(trigger_pin=9, echo_pin=11)
        ultrasonic_right = UltrasonicSensor(trigger_pin=5, echo_pin=6)
        
        auto = Controller.AutoController()
        auto_norm_left_motor_speed = auto.load_data('auto_norm_left_motor_speed.csv')
        auto_norm_right_motor_speed = auto.load_data('auto_norm_right_motor_speed.csv')
        l_motor, r_motor = initializeMotors()
        
        # Initial time.
        init_time = time.time()
        
        while True:
            # Get the GPS data for both remote and auto instances.
            try:
                [tmp_latitude, tmp_longitude, tmp_date_string, tmp_time_string] = localization.read_serial_gps()
                
                # Write GPS data to remote and autonomous array.
                if latitude != None and longitude != None:
                    tmp_latitude = latitude
                    tmp_longitude = longitude
                    tmp_date_string = date_string
                    tmp_time_string = time_string
            except:
                pass

            # Measure distances from all three sensors
            distance_left = ultrasonic_left.measure_distance()
            distance_front = ultrasonic_front.measure_distance()
            distance_right = ultrasonic_right.measure_distance()
                
            # Print distances
            print("Left Distance: {:.1f} cm".format(distance_left))
            print("Front Distance: {:.1f} cm".format(distance_front))
            print("Right Distance: {:.1f} cm".format(distance_right))
            print("--------------------------")
            
            left, right = auto.auto_controller(distance_left/150.0, distance_front/150.0, distance_right/150.0, auto_norm_left_motor_speed, auto_norm_right_motor_speed)
            print(left, right)
            # Write data to CSV file.
            writer.writerow({'Time': time.time()-init_time, 'Latitude': latitude, 'Longitude': longitude, 'Left Distance': distance_left, 'Front Distance': distance_front, 'Right Distance': distance_right, 'Left Motor Speed':left, 'Right Motor Speed':right})
            
            left =left*50
            right=right*50
            if left > 0:
                GPIO.output(20, GPIO.HIGH) # Rotate clockwise.
            else:
                GPIO.output(20, GPIO.LOW) # Rotate anticlockwise.
                
            if right > 0:
                GPIO.output(21, GPIO.LOW) # Rotate clockwise.
            else:
                GPIO.output(21, GPIO.HIGH) # Rotate anticlockwise.
                
            
            l_motor.ChangeDutyCycle(abs(left))
            r_motor.ChangeDutyCycle(abs(right))

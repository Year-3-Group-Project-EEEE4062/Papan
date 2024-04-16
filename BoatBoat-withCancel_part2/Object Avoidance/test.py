import RPi.GPIO as GPIO
import time
import Controller


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
                return 100  # Return 100 to indicate timeout.

        # Wait for the echo pulse to end.
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()
            # Timeout if echo pulse lasts longer than 0.1 second.
            if stop_time - start_time > 0.1:
                return 100  # Return 100 to indicate timeout.

        # Calculate distance.
        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound is 34300 cm/s.
        
        if distance > 100:
            return 100
        
        return distance

if __name__ == "__main__":
    # Define GPIO pins for trigger and echo
    ultrasonic_left = UltrasonicSensor(trigger_pin=8, echo_pin=7)
    ultrasonic_front = UltrasonicSensor(trigger_pin=9, echo_pin=11)
    ultrasonic_right = UltrasonicSensor(trigger_pin=5, echo_pin=6)
    
    auto = Controller.AutoController()
    auto_norm_left_motor_speed = auto.load_data('auto_norm_left_motor_speed.csv')
    auto_norm_right_motor_speed = auto.load_data('auto_norm_right_motor_speed.csv')
    
    while True:
        # Measure distances from all three sensors
        distance_left = ultrasonic_left.measure_distance()
        distance_front = ultrasonic_front.measure_distance()
        distance_right = ultrasonic_right.measure_distance()
            
        # Print distances
        print("Left Distance: {:.1f} cm".format(distance_left))
        print("Front Distance: {:.1f} cm".format(distance_front))
        print("Right Distance: {:.1f} cm".format(distance_right))
        print("--------------------------")
        
        
            
        left, right = auto.auto_controller(distance_left/100.0, distance_front/100.0, distance_right/100.0, auto_norm_left_motor_speed, auto_norm_right_motor_speed)
        print(left, right)
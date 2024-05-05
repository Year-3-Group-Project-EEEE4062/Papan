import RPi.GPIO as GPIO
import time

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
            # Timeout if no echo received within 1 second.
            if start_time - stop_time > 1:
                return 150  # Return -1 to indicate timeout.

        # Wait for the echo pulse to end.
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()
            # Timeout if echo pulse lasts longer than 1 second.
            if stop_time - start_time > 1:
                return 150  # Return -1 to indicate timeout.

        # Calculate distance.
        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound is 34300 cm/s.
        
        if distance > 150:
            return 150
        
        return distance
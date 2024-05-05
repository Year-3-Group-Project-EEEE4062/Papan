import RPi.GPIO as GPIO
import time


class BuzzerBuzzer:
    def __init__(self, pin):
        """
        Initialize the Buzzer object with the given GPIO pin.
        """
        # Set the GPIO mode.
        GPIO.setmode(GPIO.BCM)

        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)

    def buzz(self, pitch, duration):
        """
        Generate a buzz sound with the given pitch and duration.
        """
        period = 1.0 / pitch
        delay = period / 2
        cycles = int(duration * pitch)
        for _ in range(cycles):
            GPIO.output(self.pin, True)
            time.sleep(delay)
            GPIO.output(self.pin, False)
            time.sleep(delay)

    def play_melody(self):
        """
        Play a predefined melody.
        """
        melody = [
            (261, 0.3),  # C
            (293, 0.3),  # D
            (329, 0.3),  # E
            (349, 0.3)   # F
        ]
        for note in melody:
            self.buzz(note[0], note[1])

    def play_warning(self):
        """
        Play a warning signal.
        """
        warning_tone = (1000, 0.2)  # Adjust frequency and duration as needed
        self.buzz(*warning_tone)

    def cleanup(self):
        """
        Clean up GPIO resources.
        """
        GPIO.cleanup()

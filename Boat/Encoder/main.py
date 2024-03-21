import RPi.GPIO as GPIO
import time

encoder_pin = 17  # GPIO pin the encoder is connected to
pulses = 0  # Number of pulses

def setup():
    global pulses
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(encoder_pin, GPIO.FALLING, callback=counter)
    pulses = 0

def counter(channel):
    global pulses
    pulses += 1

def loop():
    while True:
        print(pulses)
        

if __name__ == '__main__':
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        GPIO.cleanup()

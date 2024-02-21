import RPi.GPIO as GPIO
import time


# GPIO pins initialization.
enA = 12   # Pin for left motor.
in1 = 20
enB = 13   # Pin for right motor.
in2 = 21

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(enA, GPIO.OUT)
GPIO.setwarnings(False)
GPIO.setup(in1, GPIO.OUT)
GPIO.setwarnings(False)

GPIO.setup(enB, GPIO.OUT)
GPIO.setwarnings(False)
GPIO.setup(in2, GPIO.OUT)
GPIO.setwarnings(False)


# Initialization for both motors.
l = GPIO.PWM(enA, 1000)  # PWM signal on pin enA with frequency of 1000Hz.
l.start(0)
r = GPIO.PWM(enB, 1000)  # PWM signal on pin enB with frequency of 1000Hz.  
r.start(0)


# Bla bla functions.
def move_forward(speed):
    # Left motor to rotate clockwise and right anticlockwise.
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    # Change the PWM speed of both motors.
    l.ChangeDutyCycle(speed)
    r.ChangeDutyCycle(speed)


def move_backward(speed):
    # Left motor to rotate clockwise and right anticlockwise.
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)

    # Change the PWM speed of both motors.
    l.ChangeDutyCycle(speed)
    r.ChangeDutyCycle(speed)
    

def move_left(speed):
    # Left motor to rotate clockwise and right anticlockwise.
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    # Change the PWM speed of both motors.
    l.ChangeDutyCycle(0)
    r.ChangeDutyCycle(speed)
    

def move_right(speed):
    # Left motor to rotate clockwise and right anticlockwise.
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    # Change the PWM speed of both motors.
    l.ChangeDutyCycle(speed)
    r.ChangeDutyCycle(0)
    

def stop():
    # Change the PWM speed of both motors.
    l.ChangeDutyCycle(0)
    r.ChangeDutyCycle(0)
    
    
# Continuous loop for controlling the boat.
while True:
    direction = input("Input direction = ")
    speed = int(input("Input speed = "))
    if (direction == 'Forward'):
        move_forward(speed)
    elif (direction == 'Backward'):
        move_forward(speed)
    elif (direction == 'Left'):
        move_forward(speed)
    elif (direction == 'Right'):
        move_forward(speed)
    
    time.sleep(1)
    stop()
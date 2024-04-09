import os
import time 
import RPi.GPIO as GPIO
import numpy as np
import serial
import base64
from mssgDecoder import decoderMssg
from autoFeature import autoClass
from remoteFeature import remoteClass
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


def newData_cb(channel):
    """
    Interrupt callback function triggered when new serial data is available to read.
    
    Args:
        channel: Represents the GPIO channel or pin number triggering the interrupt.

    Returns:
        None.
    """
    picoStr = picoW_USB.readline().decode().strip()
    converted = bytearray(base64.b64decode(picoStr))

    try:
        # Convert base64-encoded string back to bytearray.
        converted = bytearray(base64.b64decode(picoStr))
        print("############################################################")
        print("TRUE: ",picoStr)
        decoder.setNewMssg(converted)

    except:
        print("############################################################")
        print("NOT: ",picoStr)


def sendData_cb(data):
    """
    Function to send data back to the Pico W serially through USB.

    Args:
        data: List of unsigned 8-bit integers representing the message to be sent back to Medium.

    Returns:
        None.
    """
    base64_str = base64.b64encode(data).decode('utf-8')
    strToSend = base64_str + '\n'
    picoW_USB.write(strToSend.encode())


################################################################################################################
################################################################################################################
def main():
    PicoW_RX = 27   # White wire for interrupt (from Pico to Pi).

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Set up interrupt pin for USB UART with Pico W.
    GPIO.setup(PicoW_RX, GPIO.IN)
    GPIO.add_event_detect(PicoW_RX, GPIO.RISING, callback=newData_cb)

    # Class to decode messages from pico W.
    global decoder
    decoder = decoderMssg()

    # Set up serial UART with Pico W.
    global picoW_USB
    picoW_USB = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
    picoW_USB.flush()
    print("Serial flushed!!")

    # Used for changing each motor PWM.
    l_motor, r_motor = initializeMotors()

    # Create instance of GPS class for localization.
    localization = GPS()
    
    # Initial time.
    init_time = time.time()
    
    # Create instances of each feature.
    remoteInstance = remoteClass(l_motor, r_motor, sendData_cb, init_time)
    autoInstance = autoClass(l_motor, r_motor, sendData_cb, init_time)
    
    # Message from medium to Pi.
    mode = None
    mssg = None
    
    # Reset bool/flag for knowing there is a new message.
    decoder.resetFlag()
    
    # True = taking temp reading, False = not taking temp reading but still got work, None = not taking temp reading and no work.
    boatBusy = None
    # Start or stop temperature measurement (True or False).
    quitTemperature = False
    # Receiving or not receiving new mssg (True or False).
    takeReading = False
    
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
            
            latitude =  2.944418
            longitude = 101.874059
            date_string = '28/3/2024'
            time_string = '11:23'
            
            if latitude != None and longitude != None:
                remoteInstance.setGPSData(time_string, date_string, latitude, longitude)
                autoInstance.setGPSData(time_string, date_string, latitude, longitude)
                print('1st GPS reading.')
                print('Time = ' + time_string+', Date = ' + date_string + ', Latitude = ' + str(latitude) + ', Longitude = ' + str(longitude))
                break
            else:
                print("Trying to get 1st GPS reading.")
                continue
        except:
            pass

    print("Obtained GPS reading..")
    print("Program started..")
    
    
    
    #### Main while loop.
    try:
        while True:
            # Get the GPS data for both remote and auto instances.
            try:
                [latitude, longitude, date_string, time_string] = localization.read_serial_gps()
                
                # Write GPS data to remote and autonomous array.
                if latitude != None and longitude != None:
                    remoteInstance.setGPSData(time_string, date_string, latitude, longitude)
                    autoInstance.setGPSData(time_string, date_string, latitude, longitude)
            except:
                pass
                           
            ########################################################################
            # Check if there is new instruction from pico W.
            if decoder.checkIfNewMssg():
                # This means there is a new message from pico W.
                # Get the new message.
                mode, mssg = decoder.getNewMssg()
                
                # Check what mode the instruction is from
                # also make that class make decisions based whether boat busy or not.
                if mode=='r':
                    [boatBusy, quitTemperature] = remoteInstance.processRemoteMssg(mssg, boatBusy, quitTemperature)
                elif mode=='a':
                    [quitTemperature, takeReading] = autoInstance.processAutoMssg(mssg, boatBusy, quitTemperature, takeReading)
                else:
                    pass
                
                # Reset bool/flag for knowing there is a new message.
                decoder.resetFlag()

            else:
                if boatBusy:
                    # When boatBusy is True (Measuring temperature):
                    #   if quitTemperature == True : Perform temperature task (return boatBusy=False, quitTemperature=False when task end)
                    #   if quitTemperature == False: End temperature task (return quitTemperature=False, boatBusy=True, will return False when mechanical part is back to original place)
                    # Once done measuring temperature (either in remote or auto mode), change mode back to False.
                    # quitTemperature signal to start or stop temperature operation.
                    # boatBusy = temperatureMechanism(quitTemperature)
                    
                    # Assume safe to move again.
                    if quitTemperature == True:
                        boatBusy = False
                        quitTemperature = False
                    
                elif mode == 'a' and not(takeReading):
                    # Run autonomous mode when not taking new waypoints.
                    autoInstance.runAutonomous(boatBusy)
                
                else:
                    pass  
    
    except KeyboardInterrupt:
        # Stop the boat as a safety.
        remoteInstance.stop()
        print("Keyboard interrupt to stop boat.")


if __name__=="__main__":
    if os.path.exists('autoMode.csv'):
        os.remove('autoMode.csv')
    
    main()
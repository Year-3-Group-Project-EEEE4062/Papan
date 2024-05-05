import os
import time 
import RPi.GPIO as GPIO
import numpy as np
import serial
import base64
import csv
from mssgDecoder import decoderMssg
from autoFeature import autoClass
from remoteFeature import remoteClass
from FuzzyApplication import FuzzyClass
from Localization import GPS
from mssgEncoder import encoderMssg


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
    GPIO.output(in1, GPIO.LOW) # Rotate clockwise.
    l = GPIO.PWM(enA, 1000)    # PWM signal on pin enA with frequency of 1000Hz.
    l.start(0)

    # Setup right motor.
    GPIO.setup(enB, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.output(in2, GPIO.HIGH) # Rotate anticlockwise.
    r = GPIO.PWM(enB, 1000)     # PWM signal on pin enB with frequency of 1000Hz.
    r.start(0)

    return l, r

def sendData_cb(data):
    """
    Function to send data back to the Pico W serially through USB.

    Args:
        data: List of unsigned 8-bit integers representing the message to be sent back to Medium.

    Returns:
        None.
    """
    print(data)
    picoW_USB.reset_input_buffer()
    picoW_USB.reset_output_buffer()
    base64_str = base64.b64encode(data).decode('utf-8')
    strToSend = base64_str + '\n'
    picoW_USB.write(strToSend.encode())

################################################################################################################
################################################################################################################
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Class to decode messages from pico W.
    global decoder
    decoder = decoderMssg()

    # Set up serial UART with Pico W.
    global picoW_USB
    picoW_USB = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
    #picoW_USB.flush()
    picoW_USB.reset_input_buffer()
    picoW_USB.reset_output_buffer()
    print("Pico W serial flushed!!")

    # Set up serial UART with Arduino.
    global arduino_USB
    arduino_USB = serial.Serial(port="/dev/ttyACM1", baudrate=115200)
    arduino_USB.flush()
    print("Arduino serial flushed!!")

    # Used for changing each motor PWM.
    l_motor, r_motor = initializeMotors()

    # Create instance of GPS class for localization.
    localization = GPS()
    
    # Initial time.
    init_time = time.time()
    
    encoder = encoderMssg()
    
    # Create instances of each feature.
    remoteInstance = remoteClass(l_motor, r_motor, sendData_cb, init_time)
    autoInstance = autoClass(l_motor, r_motor, sendData_cb, init_time)
    fuzzyInstance = FuzzyClass(l_motor, r_motor, init_time)

    # Message from medium to Pi.
    mode = None
    mssg = None
    process = [100, 0]
    
    prevBoatBusy = None
    startTime = None

    # True = taking temp reading, False = not taking temp reading but still got work, None = not taking temp reading and no work.
    boatBusy = None
    # Start or stop temperature measurement (True or False).
    quitTemperature = False
    # Receiving or not receiving new mssg (True or False).
    takeReading = False
    ArduinoRespond = False;
    
    # GPS data.
    latitude  = None
    longitude = None
    date_string = None
    time_string = None

    tmp_latitude  = None
    tmp_longitude = None
    tmp_date_string = None
    tmp_time_string = None

    print("Attempting to get GPS reading..")
    # Try until get first gps reading.
    # Once gotten 1st gps reading, start the main while loop.
    while True:
        try:
            [tmp_latitude, tmp_longitude, tmp_date_string, tmp_time_string] = localization.read_serial_gps()
            
#             tmp_latitude =  2.944611
#             tmp_longitude = 101.874219
#             tmp_date_string = '28/3/2024'
#             tmp_time_string = '11:23'
            
            if tmp_latitude != None and tmp_longitude != None:
                latitude = tmp_latitude
                longitude = tmp_longitude
                date_string = tmp_date_string
                time_string = tmp_time_string
                
                remoteInstance.setGPSData(time_string, date_string, latitude, longitude)
                autoInstance.setGPSData(time_string, date_string, latitude, longitude)
                fuzzyInstance.setGPSData(time_string, date_string, latitude, longitude)
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
                [tmp_latitude, tmp_longitude, tmp_date_string, tmp_time_string] = localization.read_serial_gps()
                
                # Write GPS data to remote and autonomous array.
                if tmp_latitude != None and tmp_longitude != None:
                    latitude = tmp_latitude
                    longitude = tmp_longitude
                    date_string = tmp_date_string
                    time_string = tmp_time_string

                    remoteInstance.setGPSData(time_string, date_string, latitude, longitude)
                    autoInstance.setGPSData(time_string, date_string, latitude, longitude)
                    fuzzyInstance.setGPSData(time_string, date_string, latitude, longitude)
            except:
                pass
                           
            ########################################################################
            # Check if there is new instruction from pico W.
            if picoW_USB.in_waiting > 0:
                # This means there is a new message from pico W.
                # Get the new message.
                picoMssg = picoW_USB.readline().decode().strip()

                try:
                    # Convert base64-encoded string back to bytearray.
                    converted = base64.b64decode(picoMssg)
                    mode, mssg = decoder.getNewMssg(converted)
                
                    # Check what mode the instruction is from
                    # also make that class make decisions based whether boat busy or not.
                    if mode=='r':
                        prevBoatBusy = boatBusy
                        [boatBusy, quitTemperature, tmp_process] = remoteInstance.processRemoteMssg(mssg, boatBusy, quitTemperature)
                    elif mode=='a':
                        [quitTemperature, takeReading] = autoInstance.processAutoMssg(mssg, boatBusy, quitTemperature, takeReading)
                    else:
                        pass
                    
                    # Set initial variables for static and dynamic controller.
                    if (prevBoatBusy == False or prevBoatBusy == None) and boatBusy == True:
                        startTime = time_string
                        # Rest all data.
                        fuzzyInstance.resetInitData()
                        fuzzyInstance.setInitData(time_string, date_string, latitude, longitude)
                        
                    # Set initial angle and position for static or dynamic control (for remote).
                    if tmp_process[0] == 100:
                        pass
                    else:
                        process[0] = tmp_process[0]
                        process[1] = tmp_process[1]
                        # Rest all data.
                        tmp_process[0] = 100
                        tmp_process[1] = 0
                        
                        # Rest all data.
                        fuzzyInstance.resetInitData()
                        fuzzyInstance.setInitData(time_string, date_string, latitude, longitude)

                        time.sleep(0.5)
                        
                    # Reset bool/flag for knowing there is a new message.
                    picoW_USB.reset_input_buffer()

                except:
                    print("############################################################")
                    print("NOT: ",picoMssg)
                    
            else:
                # Set initial variables for static and dynamic controller.
                if (prevBoatBusy == False or prevBoatBusy == None) and boatBusy == True and mode == 'a':
                    startTime = time_string
                    # Rest all data.
                    fuzzyInstance.resetInitData()
                    fuzzyInstance.setInitData(time_string, date_string, latitude, longitude)
                
                if boatBusy:
                    fuzzyInstance.runStatic()
                    if quitTemperature:
                        if ArduinoRespond:
                            arduino_USB.write(b'\x00')
                            print("Temperature Mechanism STOP...")
                            ArduinoRespond = False;
                    else:
                        if not(ArduinoRespond):
                            arduino_USB.write(b'\x01')
                            print("Temperature Mechanism START...")
                            ArduinoRespond = True;
                    
                    # Program too fast so need delay
                    time.sleep(0.1)
                    
                    # Check if there is any message from Arduino
                    if arduino_USB.in_waiting > 0:  
                        # Read a line of message sent from Arduino
                        arduinoMssg = arduino_USB.readline().rstrip()
                        print("From Arduino: ", arduinoMssg)
                        
                        boatBusy = False
                        quitTemperature = False
                        ArduinoRespond = False
                            
                        # Check what kind of message received from Arduino
                        if(arduinoMssg==b'\x00'):
                            # This means Arduino temperature mechanism successfully stopped
                            print("Temperature Mechanism successful STOP!!!")
                            
                        else:
                            TempMssg = decoder.getNewMssg(arduinoMssg)
                            with open('TemperatureFile.csv', 'a', newline='') as csvfile:
                                fieldnames = ['Date', 'Time', 'Latitude', 'Longitude', 'Depth', 'Temperature']
                                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                                
                                # Extract data obtained from Arduino
                                tmp_depth1 = TempMssg[0]
                                tmp_temperature1 = TempMssg[1]
                                tmp_depth2 = TempMssg[2]
                                tmp_temperature2 = TempMssg[3]
                                tmp_depth3 = TempMssg[4]  
                                tmp_temperature3 = TempMssg[5]
                                
                                # Store data into csv file.
                                writer.writerow({'Date': date_string, 'Time': startTime, 'Latitude': latitude, 'Longitude': longitude, 'Depth': tmp_depth1, 'Temperature': tmp_temperature1})
                                writer.writerow({'Date': date_string, 'Time': '        ', 'Latitude': latitude, 'Longitude': longitude, 'Depth': tmp_depth2, 'Temperature': tmp_temperature2})
                                writer.writerow({'Date': date_string, 'Time': time_string, 'Latitude': latitude, 'Longitude': longitude, 'Depth': tmp_depth3, 'Temperature': tmp_temperature3})
                                
                                encodedMssg = encoder.packIntegers(0, [3])
                                sendData_cb(encodedMssg)
                else:
                    if mode == 'a' and not(takeReading):
                        prevBoatBusy = boatBusy
                        # Run autonomous mode when not taking new waypoints.
                        boatBusy = autoInstance.runAutonomous(boatBusy)
                    
                    # First element of process variable store mode (1 for F and 2 for B)
                    elif mode == 'r' and (process[0] == 1 or process[0] == 2):
                        fuzzyInstance.runDynamic(process[0], process[1])
                    
                    # First element of process variable store mode (3 for static control)
                    elif mode == 'r' and (process[0] == 3):
                        fuzzyInstance.runStatic()

                    else:
                        pass  
    
    except KeyboardInterrupt:
        # Stop the boat as a safety.
        remoteInstance.stop()
        print("Keyboard interrupt to stop boat.")
        
    
main()
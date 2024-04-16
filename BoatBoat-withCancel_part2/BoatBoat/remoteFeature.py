import RPi.GPIO as GPIO
import time
from mssgEncoder import encoderMssg

class remoteClass:
    def __init__(self, l_motor, r_motor, send_cb, init_time):
        """
        Initialize the remote control class.

        Args:
            l_motor: Left motor PWM object.
            r_motor: Right motor PWM object.
            send_cb: Callback function to send data back to the controlling device.

        Returns:
            None.
        """
        self.init_time = init_time
        
        # GPS data storage.
        self.time_string = None
        self.date_string = None
        self.latitude = None
        self.longitude = None

        # Motors initialization.
        self.l_motor = l_motor
        self.r_motor = r_motor
        self.in1 = 20  # Motor control pin.
        self.in2 = 21  # Motor control pin.

        # Medium initialization.
        self.send_cb = send_cb  # Callback function for sending data back.
        self.encoder = encoderMssg()  # Encoder object for message encoding.

    def setGPSData(self, time_string, date_string, latitude, longitude):
        """
        Update GPS data.

        Args:
            time_string: Time string.
            date_string: Date string.
            latitude: Latitude data.
            longitude: Longitude data.

        Returns:
            None.
        """
        self.time_string = time_string
        self.date_string = date_string
        self.latitude = latitude
        self.longitude = longitude

    def processRemoteMssg(self, remoteMssg, boatBusy, temperatureOperation):
        """
        Process the remote control message.

        Args:
            remoteMssg: Message received from the remote control.
            boatBusy: Boolean indicating if the boat is busy.

        Returns:
            Boolean: Boat busy state, start or stop temperature measurement.
        """
        if(len(remoteMssg) == 1):
            if(remoteMssg[0] == 0):
                print("User wants to stop the boat!!")
            elif(remoteMssg[0] == 1):
                print("User wants to measure temperature!!")
                boatBusy = True
            elif(remoteMssg[0] == 2 and boatBusy == True):  # Cancel temperature measuring task.
                print("User cancels temperature measuring task!!")
                temperatureOperation = True
            # elif(remoteMssg[0] == 2):
            #     print("User cancels operation!!")
            else:
                # Normally happen when user spam the button.
                print("Invalid remote message")
                
            self.stop()

        elif(len(remoteMssg) == 2 and (boatBusy == False or boatBusy == None)):
            print("User using remote movements!")

            # Extract the speed.
            speed = remoteMssg[1]

            # Check for movement only.
            if(remoteMssg[0] == 0):
                print("Boat moves forward!")
                self.moveForward(speed)
            elif(remoteMssg[0] == 1):
                print("Boat moves backwards!")
                self.moveBackward(speed)
            elif(remoteMssg[0] == 2):
                print("Boat moves rightwards!")
                self.moveRight(speed)
            elif(remoteMssg[0] == 3):
                print("Boat moves leftwards!")
                self.moveLeft(speed)

        else:
            print("Invalid remote message")
            self.stop()

        return boatBusy, temperatureOperation

    def stop(self):
        """
        Stop the boat.

        Returns:
            None.
        """
        # Stop both motors
        self.l_motor.ChangeDutyCycle(0)
        self.r_motor.ChangeDutyCycle(0)
        print("Successfully stopped!!")

    def moveForward(self, speed):
        """
        Move the boat forward.

        Args:
            speed: Speed of the boat movement.

        Returns:
            None.
        """
        # Set motor directions
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

        # Set PWM speeds for both motors
        self.l_motor.ChangeDutyCycle(speed)
        self.r_motor.ChangeDutyCycle(speed)
        print("Successfully moved forward!!")
        
        
        
    def moveBackward(self, speed):
        """
        Move the boat backward.

        Args:
            speed: Speed of the boat movement.

        Returns:
            None.
        """
        # Set motor directions
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

        # Set PWM speeds for both motors
        self.l_motor.ChangeDutyCycle(speed)
        self.r_motor.ChangeDutyCycle(speed)
        print("Successfully moved backward!!")
        
        

    def moveRight(self, speed):
        """
        Move the boat rightwards.

        Args:
            speed: Speed of the boat movement.

        Returns:
            None.
        """
        # Set motor directions
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

        # Set PWM speed for the left motor
        self.l_motor.ChangeDutyCycle(speed)

        # Stop the right motor
        self.r_motor.ChangeDutyCycle(0)
        print("Successfully moved right!!")
        

    def moveLeft(self, speed):
        """
        Move the boat leftwards.

        Args:
            speed: Speed of the boat movement.

        Returns:
            None.
        """
        # Set motor directions
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

        # Stop the left motor
        self.l_motor.ChangeDutyCycle(0)

        # Set PWM speed for the right motor
        self.r_motor.ChangeDutyCycle(speed)
        print("Successfully moved left!!")
        
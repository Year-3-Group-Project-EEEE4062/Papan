import math
import numpy as np
import csv


class BoatController:
    """Controller for the autonomous boat."""
    def __init__(self):
        pass
                                                                                                                                    
    def load_csv(self, filename):
        """
        Load data from a CSV file.
        
        Args:
            filename (str): Name of the CSV file.
        
        Returns:
            list: A 2D numpy array containing the data from the CSV file.
        """
        data = []
        with open(filename, newline='') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                data.append([float(x) for x in row])
        return np.array(data)
    
    def calculate_distance(self, targetX, x, targetY, y):
        """
        Calculate the distance between two points in 2D space.
        
        Args:
            targetX (float): Target x-coordinate.
            x (float): Actual x-coordinate.
            targetY (float): Target y-coordinate.
            y (float): Actual y-coordinate.
        
        Returns:
            float: Distance between the two points.
        """
        deltaX = x - targetX
        deltaY = y - targetY
        sum_of_squares = deltaX * deltaX + deltaY * deltaY
        distance = math.sqrt(sum_of_squares)
        return distance

    def calculate_yaw_angle(self, Ux, Uy):
        """
        Calculate the heading angle of the boat from positive y-axis.
        
        Args:
            Ux (float): Control action in x direction.
            Uy (float): Control action in y direction.
        
        Returns:
            float: Heading angle for the boat.
        """
        absUx = abs(Ux)
        absUy = abs(Uy)
        angle_in_radians = math.atan2(absUy, absUx)
        angle_in_degrees = angle_in_radians * (180.0 / math.pi)

        if Ux >= 0 and Uy >= 0:
            angle_in_degrees = 90 - angle_in_degrees       # Quadrant 1.
        elif Ux < 0 and Uy >= 0:
            angle_in_degrees = angle_in_degrees - 90       # Quadrant 2.
        elif Ux < 0 and Uy < 0:
            angle_in_degrees = -1 * angle_in_degrees - 90  # Quadrant 3.
        elif Ux >= 0 and Uy < 0:
            angle_in_degrees = angle_in_degrees + 90       # Quadrant 4.
        else:
            angle_in_degrees = 0

        return angle_in_degrees

    def outer_controller(self, setpointX, setpointY, sensorX, sensorY):
        """
        Calculate the desired heading for the boat.
        
        Args:
            setpointX (float): Targeted x direction.
            setpointY (float): Targeted y direction.
            sensorX (float): Actual x direction.
            sensorY (float): Actual y direction.
        
        Returns:
            float: Desired heading for the boat.
        """
        Ux = setpointX - sensorX
        Uy = setpointY - sensorY
        setpoint_heading = self.calculate_yaw_angle(Ux, Uy)
        return setpoint_heading

    def inner_controller(self, distance, setpoint_yaw, yaw, norm_left_motor_speed, norm_right_motor_speed):
        """
        Calculate the normalized speed for left and right motor based on distance and yaw error.

        Args:
            distance (float): Distance between targeted and actual point.
            setpoint_yaw (float): Desired heading.
            yaw (float): Actual heading.
            norm_left_motor_speed (list of lists): 2D array containing normalized speeds for the left motor.
            norm_right_motor_speed (list of lists): 2D array containing normalized speeds for the right motor.

        Returns:
            list: A list containing normalized speeds for left and right motor.
        """
        error_yaw = setpoint_yaw - yaw
        column = 50 * distance
        row = 25 + 25 * error_yaw
        row = int(max(0, min(50, row)))
        column = int(max(0, min(50, column)))

        norm_speed = [0, 0]
        norm_speed[0] = norm_left_motor_speed[column][row]
        norm_speed[1] = norm_right_motor_speed[column][row]

        return norm_speed
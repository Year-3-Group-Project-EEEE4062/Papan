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
    
    def calc_angle(self, angle1, angle2):
        """
        Calculate the absolute angle between two angles.

        Args:
            angle1 (float): First angle in degrees.
            angle2 (float): Second angle in degrees.

        Returns:
            float: Absolute angle between the two angles in degrees.
        """
        # Calculate the difference between the angles.
        angle_diff = angle2 - angle1
        
        # Adjust for clockwise and anticlockwise directions.
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        return -1*angle_diff
    
    def fuzzy_controller(self, distance, setpoint_yaw, yaw, norm_left_motor_speed, norm_right_motor_speed):
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
        column = 100 * distance
        row = 50 + 50 * error_yaw
        row = int(max(0, min(100, row)))
        column = int(max(0, min(100, column)))

        norm_speed = [0, 0]
        norm_speed[0] = norm_left_motor_speed[column][row]
        norm_speed[1] = norm_right_motor_speed[column][row]

        return norm_speed
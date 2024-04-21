import math
import numpy as np
import csv
import time


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
    

class AutoController:
    """Object Avoidance Controller for the autonomous boat."""
    def __init__(self):
        # Initialize the number of slices in the 3D array.
        self.num_slices = 51
        
    def load_data(self, filename):
        """
        Load data from a CSV file and reshape it into a 3D array.

        Parameters:
        filename (str): Path to the CSV file.

        Returns:
        numpy.ndarray: 3D array containing the loaded data.
        """
        # Read the CSV file into a 2D array.
        flattened_data = np.loadtxt(filename, delimiter=',')

        # Determine the original dimensions of the 3D array.
        num_rows, num_cols_times_num_slices = flattened_data.shape

        # Determine the number of rows and columns for the reshaped array.
        num_cols = num_cols_times_num_slices // self.num_slices
        
        # Reshape the 2D array back into a 3D array with varying depth.
        data_restored = np.reshape(flattened_data, (num_rows, num_cols, self.num_slices))

        # Create a single 3D numpy array to store all 2D arrays with varying depth.
        array_list = []
        for i in range(self.num_slices):
            # Reshape the 1D array into a 2D array with rows and columns.
            num_rows = 51
            num_cols = 51
            data_2d = np.reshape(data_restored[:, :, i], (num_rows, num_cols)).T
            array_list.append(data_2d)
        
        # Stack the 2D arrays along a new axis to create the 3D array.
        data_3d = np.stack(array_list, axis=0)

        return np.array(data_3d)

    def auto_controller(self, dl, df, dr, auto_norm_left_motor_speed, auto_norm_right_motor_speed):
        """
        Compute normalized motor speeds based on input values and loaded data.

        Parameters:
        dl (float): Input value for dl (from 0.0 to 1.0).
        df (float): Input value for df (from 0.0 to 1.0).
        dr (float): Input value for dr (from 0.0 to 1.0).
        auto_norm_left_motor_speed (numpy.ndarray): 3D array containing left motor speed data.
        auto_norm_right_motor_speed (numpy.ndarray): 3D array containing right motor speed data.

        Returns:
        list: A list containing the normalized motor speeds for left and right motors.
        """
        # Compute the index for depth, row, and column based on input values.
        row = int(dl * 50)
        column = int(df * 50)
        depth = int(dr * 50)

        # Access normalized motor speeds based on computed indices.
        norm_speed = [0, 0]
        norm_speed[0] = auto_norm_left_motor_speed[depth, row, column]
        norm_speed[1] = auto_norm_right_motor_speed[depth, row, column]

        return norm_speed
import Controller
import Map

# Maximum speed for both motors.
max_Speed = 100


# Create an instance for class.
boat_controller = Controller.BoatController()


# Load the normalized motor speed data from CSV files.
norm_left_motor_speed = boat_controller.load_csv("norm_left_motor_speed.csv")
norm_right_motor_speed = boat_controller.load_csv("norm_right_motor_speed.csv")


# Continuous loop for controlling the boat.
while True:
    # Calculate normalized speeds for left and right motors using inner controller.
    [left, right] = boat_controller.inner_controller(0.69, 0, 0, norm_left_motor_speed, norm_right_motor_speed)
    
    # Print the calculated speeds for left and right motors.
    left *= max_Speed
    right *= max_Speed
    print("Left Motor Speed =", left, ", Right Motor Speed =", right)

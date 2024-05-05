import Controller
import Map

# Maximum speed for both motors.
max_Speed = 100


# Create an instance for class.
boat_controller = Controller.BoatController()


# Load the normalized motor speed data from CSV files.
norm_left_motor_speed = boat_controller.load_csv("norm_left_motor_speed.csv")
norm_right_motor_speed = boat_controller.load_csv("norm_right_motor_speed.csv")


# # Continuous loop for controlling the boat.
# while True:
#     # Calculate normalized speeds for left and right motors using inner controller.
#     [left, right] = boat_controller.inner_controller(0.69, 0, 0, norm_left_motor_speed, norm_right_motor_speed)
    
#     # Print the calculated speeds for left and right motors.
#     left *= max_Speed
#     right *= max_Speed
#     print("Left Motor Speed =", left, ", Right Motor Speed =", right)


# Create a GridOccupancyMap object with width and height of 21x21.
grid_occupancy_map = Map.GridOccupancyMap(21, 21)

# Update specific cells on the grid to represent obstacles or free spaces.
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(-9, -9)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True)   
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(-8, -8)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(-8, -7)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(-6, -6)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(-5, -6)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(3, 3)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True)
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(3, 4)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True)   
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(3, 5)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(4, 5)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(4, 6)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(5, 5)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True) 
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(5, 6)    
grid_occupancy_map.update_cell(array_x, array_y, obstacle=True)  

# Define start and goal positions in array coordinates.
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(-10, -10)  # Map axis coordinates to array indices.
start = (array_x, array_y)  # Define start position.
[array_x, array_y] = grid_occupancy_map.map_axis_to_array(10, 10)   # Map axis coordinates to array indices.
goal = (array_x, array_y)  # Define goal position.

# Find path using the A* algorithm.
path_array = grid_occupancy_map.a_star(start, goal)

# Convert the path from array coordinates to axis coordinates
axis_path = []
for i in path_array:
    axis = grid_occupancy_map.map_array_to_axis(i[0], i[1])
    axis_path.append(axis)

# Display the grid with the path
grid_occupancy_map.display_grid_occupancy_map(axis_path)

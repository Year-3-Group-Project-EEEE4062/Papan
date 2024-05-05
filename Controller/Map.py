import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue


class GridOccupancyMap:
    """
    Represents a grid occupancy map with obstacles.
    """

    def __init__(self, width, height):
        """
        Initializes the grid occupancy map with specified width and height.

        Args:
            width (int): The width of the grid.
            height (int): The height of the grid.
        """
        self.width = width
        self.height = height

        # Initialize the grid with -1, where 1 represents free cells, 100 represents obstacles, and -1 represents unknown cells.
        self.grid = np.full((height, width), -1, dtype=int)  # By default, all cells are unknown (-1).

        # Create a lookup table for mapping array position to actual axis position
        self.lookup_table = self.create_lookup_table()

    def create_lookup_table(self):
        """
        Creates a lookup table for mapping array position to actual axis position.

        Returns:
            numpy.ndarray: A lookup table containing the mapped coordinates for each cell in the grid.
        """
        lookup_table = np.zeros((self.height, self.width, 2), dtype=float)
        for x in range(-1*(self.width//2), self.width//2 + 1):
            for y in range(-1*(self.height//2), self.height//2 + 1):
                # Map the array position to the actual axis position.
                axis_x = x  # Adjust this calculation based on your specific mapping requirements.
                axis_y = y  # Adjust this calculation based on your specific mapping requirements.
                lookup_table[y + self.height // 2, x + self.width // 2] = [axis_y, axis_x]
        return lookup_table

    def map_array_to_axis(self, array_x, array_y):
        """
        Maps the array position to the actual axis position.

        Args:
            array_x (int): The x-coordinate in the array.
            array_y (int): The y-coordinate in the array.

        Returns:
            tuple: The mapped x and y coordinates in the actual axis.
        """
        [y, x] = self.lookup_table[array_y, array_x]
        return x, y
    
    def map_axis_to_array(self, axis_x, axis_y):
        """
        Maps the axis position to the array position.

        Args:
            axis_x (int): The x-coordinate in the actual axis.
            axis_y (int): The y-coordinate in the actual axis.

        Returns:
            tuple: The mapped x and y coordinates in the array.
        """
        # Adjust the axis position to align with array indices.
        array_x = axis_x + (self.width // 2)
        array_y = axis_y + (self.height // 2)

        return array_x, array_y


    def update_cell(self, array_x, array_y, obstacle=False):
        """
        Updates the occupancy status of a cell at position (array_x, array_y).

        Args:
            array_x (int): The x-coordinate of the cell in the array.
            array_y (int): The y-coordinate of the cell in the array.
            obstacle (bool): Whether the cell is an obstacle (True) or not (False).
            quadrant (int): The quadrant in which the cell is located (1, 2, 3, or 4).
        """

        if (0 <= array_x < self.width) and (0 <= array_y < self.height):
            if not obstacle:
                self.grid[array_y, array_x] = 0  # Mark cell as free.
            else:
                self.grid[array_y, array_x] = 100  # Mark cell as obstacle in Q1, Q2, Q3 or Q4.

    def is_valid_cell(self, array_x, array_y):
        """
        Checks if a cell at position (x, y) is valid (within bounds and not an obstacle).

        Args:
            x (int): The x-coordinate of the cell.
            y (int): The y-coordinate of the cell.

        Returns:
            bool: True if the cell is valid, False otherwise.
        """
        return 0 <= array_x < self.width and 0 <= array_y < self.height

    def heuristic(self, start, goal):
        """
        Calculates the heuristic (Euclidean distance) between two points.

        Args:
            start (tuple): The start point coordinates (x, y).
            goal (tuple): The goal point coordinates (x, y).

        Returns:
            float: The Euclidean distance between the start and goal points.
        """
        return np.linalg.norm(np.array(start) - np.array(goal))  # Calculate the Euclidean distance between two points.

    def a_star(self, start, goal):
        """
        Finds the shortest path from start to goal using A* algorithm.

        Args:
            start (tuple): The start point array (array_x, array_y).
            goal (tuple): The goal point coordinates (array_x, array_y).

        Returns:
            list: The list of array positions representing the path from start to goal.
                Returns None if no path is found.
        """
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {pos: float('inf') for pos in np.ndindex(self.grid.shape)}
        g_score[start] = 0

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                next_pos = (current[0] + dx, current[1] + dy)
                if not (0 <= next_pos[0] < self.width and 0 <= next_pos[1] < self.height):
                    continue
                if self.grid[next_pos[1], next_pos[0]] == 100:  # Check if next position is an obstacle
                    continue
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score[next_pos]:
                    came_from[next_pos] = current
                    g_score[next_pos] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(next_pos, goal)
                    open_set.put((f_score, next_pos))

        return None  # No path found.

    
    def display_grid_occupancy_map(self, path):
        """
        Displays the grid occupancy map.
        """
        fig, ax = plt.subplots(figsize=(10, 10))

        # Set title and labels.
        ax.set_title('Grid Occupancy Map')
        ax.set_xlabel('x-axis (m)')
        ax.set_ylabel('y-axis (m)')

        # Iterate over each grid cell.
        for array_x in range(self.width):
            for array_y in range(self.height):
                # Coordinates of the corners of the grid cell.
                [x, y] = self.map_array_to_axis(array_x, array_y)
                min_x, min_y = x, y
                max_x, max_y = x + 1, y + 1
                corners = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]

                if self.grid[array_y, array_x] == 100:  # If obstacle, color black.
                    ax.add_patch(plt.Polygon(corners, color='black'))
                elif self.grid[array_y, array_x] == 0:  # If free, color white.
                    ax.add_patch(plt.Polygon(corners, color='white', alpha=0.5))
                elif self.grid[array_y, array_x] == -1: # If unknown, color light grey.
                    ax.add_patch(plt.Polygon(corners, color='lightgrey', alpha=0.5))

        # Set white background.
        fig.patch.set_facecolor('white')

        # Equal aspect ratio.
        ax.set_aspect('equal')

        # Get the actual minimum and maximum axis coordinates.
        min_x, min_y = self.map_array_to_axis(0, 0)
        max_x, max_y = self.map_array_to_axis(self.width - 1, self.height - 1)

        # Set the x-axis tick interval.
        plt.xticks(np.arange(-max_x, max_x+2, 1))
        # Set the y-axis tick interval.
        plt.yticks(np.arange(-max_y, max_y+2, 1))

        # Plot path if provided.
        if path:
            path_arr = np.array(path)
            path_x, path_y = path_arr[:, 0], path_arr[:, 1]
            ax.plot(path_x, path_y, color='blue', marker='x', markersize=5, linestyle='-', linewidth=1)  # Plot path as blue line with 'x' markers
        else:
            print("No path found")

        plt.show()
import numpy as np

OCCUPANCY_THRESHOLD = 50  

class Grid:
    """Adapted from the Grid example code provided in class.
         Converts an OccupancyGrid into a 2D matrix. Encapsulates all functions pertaining to the grid itself"""
    def __init__(self, grid_data, width, height, resolution):
        self.grid = np.reshape(grid_data, (height, width))
        self.width = width
        self.height = height
        self.resolution = resolution

    def cell_at(self, x, y):
        """Returns value of cell at x,y"""
        # print("cell at " + str(x) + " " + str(y) + " " + str(self.grid[y, x]))
        return self.grid[y, x]

    def cell_to_point(self, col, row):
        """Given the row and column location within the grid, return the corresponding location in the map reference frame"""
        x = self.resolution * col  # multiply cell size by number of columns in the grid, then add to origin
        y = self.resolution * row  # multiply cell size by number of rows in the grid, then add to origin
        p = [x, y]

        return p

    def point_to_cell(self, x, y):
        """Given the current x and y position within the grid, return the corresponding cell (row and col) values"""
        pos_x = int(x / self.resolution)
        pos_y = int(y / self.resolution)
        cell = (pos_x, pos_y)

        return cell

    def is_occupied(self, col, row):
        """This method determines if a specified cell is occupied or not. True if occupied, false if not"""
        return self.cell_at(col, row) > OCCUPANCY_THRESHOLD

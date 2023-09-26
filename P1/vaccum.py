"""
implement the logic of a navigation algorithm for an autonomous vacuum cleaner by making use of the location of the robot.
The robot is equipped with a map and knows it’s current location in it.
The main objective will be to cover the largest area of ​​a house using the programmed algorithm

Our solution will be a complete coverage BSA (backtracking spiral algorithm).
This algorithm divides the map into a grid.
the algorithm is:
    try to move east/south/west/north (if you can't in one direction because there's an obstacle, try in another direction)
    if you are surrounded by obstacles, move to the closest empty square
    every square you have visited is a "virtual obstacle" (marked as not dirty)

The map will be generated from an image, where the black pixels will be obstacles and the white pixels will be free spaces.
The robot will be able to move in 4 directions (up, down, left, right) and will have a sensor that will allow it to detect obstacles.
"""

import numpy as np
import time
from PIL import Image
class MapCell:
    def __init__(self, position, is_obstacle):
        self.position = position
        self.tags = {}
        if(is_obstacle):
            self.tags["obstacle"] = True
            self.tags["dirty"] = False
        else:
            self.tags["obstacle"] = False
            self.tags["dirty"] = True
        self.tags["return_point"] = False
        self.tags["robot"] = False

    def __str__(self):
        text=""
        if(self.tags["robot"]):
            text += "_"
        elif(self.tags["obstacle"]):
            text += "*"
        elif(self.tags["dirty"]):
            text += "D"
        else:
            text += "C"
        return text

    def edit_tag(self, tag, value):
        self.tags[tag] = value



class MapGrid:
    def __init__(self, image_data, grid_size):
        self.width = image_data.shape[0]//grid_size
        self.height = image_data.shape[1]//grid_size
        self.grid=[["." for x in range(self.height)] for y in range(self.width)]
        for x in range(self.width):
            for y in range(self.height):
                #get the pixels of the cell
                cell_pixels = image_data[x*grid_size:(x+1)*grid_size, y*grid_size:(y+1)*grid_size]
                #check if the cell is an obstacle
                is_obstacle = np.any(cell_pixels == 0)
                #create the cell
                self.grid[x][y] = MapCell((x, y), is_obstacle)

    def __str__(self):
        text=f"Width: {self.width}, Height: {self.height}\n"
        for row in self.grid:
            for cell in row:
                text += f"{cell} "
            text += "\n"
        return text
    
    def get_neighbors(self, position):
        """
        return the neighbors of a cell
        """
        neighbors = []
        if(position[1] < self.height-1):
            neighbors.append((position[0], position[1]+1))
        if(position[0] < self.width-1):
            neighbors.append((position[0]+1, position[1]))
        if(position[1] > 0):
            neighbors.append((position[0], position[1]-1))
        if(position[0] > 0):
            neighbors.append((position[0]-1, position[1]))

        return neighbors
    
    def closest_dirty_cell(self,position):
        """
        return the closest dirty cell
        for each dirty cell in the map, calculate the distance to the current cell, use the manhattan distance
        sort the list of cells by distance
        return the closest dirty cell
        """
        dirty_cells = []
        for x in range(self.width):
            for y in range(self.height):
                if(self.grid[x][y].tags["dirty"] and self.grid[x][y].tags["return_point"]):
                    distance = np.sqrt((x-position[0])**2 + (y-position[1])**2)
                    dirty_cells.append((distance, (x,y)))
        dirty_cells.sort(key=lambda x: x[0])
        return dirty_cells[0][1]
        

    
class Robot:
    def __init__(self, map, position):
        self.map = map
        self.position = position
        self.actions = []
        map.grid[position[0]][position[1]].edit_tag("robot", True)

    def step(self):
        neighbors = self.map.get_neighbors(self.position)
        #check if there are dirty cells
        dirty_cells = []
        for neighbor in neighbors:
            if(self.map.grid[neighbor[0]][neighbor[1]].tags["dirty"]):
                dirty_cells.append(neighbor)
        if(len(dirty_cells) > 0):
            #move to the first dirty cell
            self.move(dirty_cells[0])
            return
        #the robot is stuck, move to the closest dirty cell
        dirty_cells.append(self.map.closest_dirty_cell(self.position))
        self.move(dirty_cells[0])
        return
    def move(self, position):
        #clean the current cell
        self.map.grid[self.position[0]][self.position[1]].edit_tag("dirty", False)
        #set the neighbors of the current cell as return points if they are dirty
        neighbors = self.map.get_neighbors(self.position)
        for neighbor in neighbors:
            if(self.map.grid[neighbor[0]][neighbor[1]].tags["dirty"]):
                self.map.grid[neighbor[0]][neighbor[1]].edit_tag("return_point", True)

        #remove the robot from the current cell
        self.map.grid[self.position[0]][self.position[1]].edit_tag("robot", False)
        #move the robot
        self.position = position
        #add the robot to the new cell
        self.map.grid[self.position[0]][self.position[1]].edit_tag("robot", True)

    

if __name__ == "__main__":
    #open image
    #image = Image.open("map.png")
    with Image.open("map.png") as image:
        #convert image to numpy array
        image_data = np.asarray(image)
    #create map grid
    map = MapGrid(image_data, 20)
    #create robot
    robot = Robot(map, (3,2))
    for i in range(1000):
        robot.step()
        print(map)
        time.sleep(0.2)
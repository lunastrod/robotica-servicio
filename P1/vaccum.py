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
    
    def path_distance(self, start, end):
        """
        return the distance between two cells
        sqrt((x1-x2)^2 + (y1-y2)^2)
        """
        return ((start[0]-end[0])**2 + (start[1]-end[1])**2)**0.5
    
    def lowest_f_score(self, open_set, f_score):
        """
        return the cell with the lowest f_score
        f_score is the sum of g_score and the heuristic
        g_score is the distance from the start cell to the current cell
        """
        lowest_f_score = None
        for cell in open_set:
            if(lowest_f_score == None):
                lowest_f_score = cell
            elif(f_score[cell] < f_score[lowest_f_score]):
                lowest_f_score = cell
        return lowest_f_score
    
    def path_to(self, start, end):
        """
        return the path from start to end
        the algorithm is A*, the heuristic is the manhattan distance
        g_score is the distance from the start cell to the current cell
        f_score is the sum of g_score and the heuristic
        """
        #print(f"start: {start}, end: {end}")
        open_set = set()
        open_set.add(start)
        came_from = {}
        g_score = {}
        g_score[start] = 0
        f_score = {}
        f_score[start] = self.path_distance(start, end)
        while(len(open_set) > 0):
            #get the cell with the lowest f_score
            current = self.lowest_f_score(open_set, f_score)
            if(current == end):
                #reconstruct the path
                path = []
                while(current in came_from):
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            open_set.remove(current)
            neighbors = self.get_neighbors(current)
            for neighbor in neighbors:
                if(self.grid[neighbor[0]][neighbor[1]].tags["obstacle"]):
                    continue
                tentative_g_score = g_score[current] + 1
                if(neighbor not in g_score or tentative_g_score < g_score[neighbor]):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.path_distance(neighbor, end)
                    if(neighbor not in open_set):
                        open_set.add(neighbor)
        return None
        
    def closest_dirty_cell(self,position):
        """
        return the closest dirty cell
        for each dirty cell in the map, calculate the distance to the current cell, using the A* algorithm
        return the path to the closest dirty cell
        don't sort the list, get the minimum value
        """
        min_distance = 10000000
        shortest_path = []
        for x in range(self.width):
            for y in range(self.height):
                if(self.grid[x][y].tags["dirty"]):
                    if(self.path_distance(position, (x,y)) > min_distance):
                        continue
                    path=self.path_to(position, (x,y))
                    if(min_distance > len(path)):
                        min_distance = len(path)
                        shortest_path = path
        return shortest_path

        
        

    
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
        path=self.map.closest_dirty_cell(self.position)
        for position in path:
            self.move(position)
        return
    
    def move(self, position):
        #clean the current cell
        self.map.grid[self.position[0]][self.position[1]].edit_tag("dirty", False)
        #remove the robot from the current cell
        self.map.grid[self.position[0]][self.position[1]].edit_tag("robot", False)
        #move the robot
        self.position = position
        #add the robot to the new cell
        self.map.grid[self.position[0]][self.position[1]].edit_tag("robot", True)

        time.sleep(0.02)
        print(map)

    

if __name__ == "__main__":
    #open image
    #image = Image.open("map.png")
    with Image.open("map.png") as image:
        #convert image to numpy array
        image_data = np.asarray(image)
    #create map grid
    map = MapGrid(image_data, 15)
    #create robot
    robot = Robot(map, (5,5))
    for i in range(1000):
        robot.step()
        
"""
26-sep: I started with my navigation algorithm. I made 3 classes
class MapGrid: it represents the map, it has a grid of cells
class MapCell: it represents a cell, it has a position and tags
class Robot: it represents the robot, it has a position and a map

The map is generated from an image, where the black pixels are obstacles and the white pixels are free spaces.
The robot can move in 4 directions (up, down, left, right).
When the robot moves, it cleans the current cell and moves to a dirty neighbour cell.
The robot tends to form a spiral, but it eventually gets stuck.
If there are no dirty neighbour cells, the robot moves to the closest dirty cell.
The closest dirty cell is calculated using the A* algorithm.
The robot moves to the closest dirty cell using the shortest path dictated by the A* algorithm.
"""
        
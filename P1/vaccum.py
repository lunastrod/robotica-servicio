
UNIBOTICS = False
if(UNIBOTICS):
    from GUI import GUI
    from HAL import HAL

import numpy as np
import time
from PIL import Image
import cv2
import math
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
        self.grid_size = grid_size
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
    
    def draw_map(self, image):
        """
        draw the map on top of the image
        """
        new_image = image.copy()
        for i in range(self.height):
            for j in range(self.width):
                pixel_coords = self.grid_to_pixel_coords((j,i))#warning, the x and y are inverted
                if(self.grid[i][j].tags["obstacle"]):
                    cv2.rectangle(new_image, (pixel_coords[0]-self.grid_size//3, pixel_coords[1]-self.grid_size//3), (pixel_coords[0]+self.grid_size//3, pixel_coords[1]+self.grid_size//3), (0,0,127), -1)
                else:
                    cv2.rectangle(new_image, (pixel_coords[0]-self.grid_size//3, pixel_coords[1]-self.grid_size//3), (pixel_coords[0]+self.grid_size//3, pixel_coords[1]+self.grid_size//3), (127,0,0), -1)
        #paint the origin of the real coordinates
        cv2.circle(new_image, self.real_to_pixel_coords((0,0)), 5, (0,255,0), -1)
        cv2.circle(new_image, self.real_to_pixel_coords((-1,1.5)), 5, (0,255,0), -1)
        return new_image

    
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

    def real_to_pixel_coords(self, position):
        """
        convert the real coordinates to map coordinates
        """
        #the map is 10x10 meters
        #scale
        x = position[0]*image_data.shape[0]//10
        y = position[1]*image_data.shape[1]//10
        x = -x #flip the x axis
        #translate
        x = x+580
        y = y+420
        return (round(x),round(y))

    def grid_to_pixel_coords(self, position):
        """
        convert the map coordinates to pixel coordinates
        """
        x=self.grid_size*position[0]+self.grid_size//2
        y=self.grid_size*position[1]+self.grid_size//2
        return (x,y)
    
    def grid_to_real_coords(self, position):
        """
        convert grid coords to real coords
        """
        x=self.grid_size*position[0]+self.grid_size//2
        y=self.grid_size*position[1]+self.grid_size//2
        #now we have pixel coords. convert to real coords translating and scaling
        x = x-580
        y = y-420
        x = -x
        x = x*10/image_data.shape[0]
        y = y*10/image_data.shape[1]
        return (x,y)

    def real_to_grid_coords(self, position):
        """
        convert real coords to grid coords
        """
        #convert to pixel coords
        x,y=self.real_to_pixel_coords(position)
        #convert to grid coords
        x = round(x//self.grid_size)
        y = round(y//self.grid_size)
        return (x,y)
        


class RobotPlanner:
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
        #add the action to the list
        self.actions.append(position)
        if(not UNIBOTICS):
            pass
            #print(map)
            #time.sleep(0.2)

class RobotController:
    def __init__(self, plan):
        self.points = plan

    def VFF_controller(self, goal):
        #parameters
        goal_k = 2
        max_goal_force = 2.6
        #update the goal force
        goal_mod = math.sqrt(goal[0]*goal[0]+goal[1]*goal[1])
        unit_goal = [goal[0]/goal_mod, goal[1]/goal_mod]
        goal_strength = min(max_goal_force, goal_k*goal_mod)
        goal_force = [unit_goal[0]*goal_strength, unit_goal[1]*goal_strength]
        return goal_force
    
    def force_to_vw(self, force):
        maxv = 1
        maxw = 3
        kv = 2
        kw = 10

        angle = math.atan2(-force[1], force[0])
        #print("angle",angle)
        v=maxv/(1+kv*angle**2)-0.1685
        w=max(-maxw, min(maxw, kw*angle**3))
        return v, w
    
    def is_close(self, goal):
        threshold = 0.1
        if(abs(goal[0]) < threshold and abs(goal[1]) < threshold):
            return True
        return False
    
    def local_coords(self, point, position, angle):
        """
        return the local coordinates of a point
        """
        x = point[0]-position[0]
        y = point[1]-position[1]
        x2 = x*math.cos(angle) + y*math.sin(angle)
        y2 = -x*math.sin(angle) + y*math.cos(angle)
        return (x2, y2)
    
    def step(self, position, angle):
        """
        return the linear and angular velocity of the robot
        """
        if(len(self.points)==0):
          return 0,0
        #get the current point
        point = self.points[0]
        #get the local coordinates of the point
        local_point = self.local_coords(point, position, angle)
        #print("local_point",local_point)
        #get the goal force
        goal_force = self.VFF_controller(local_point)
        #get the linear and angular velocity
        v, w = self.force_to_vw(goal_force)
        #check if the robot is close to the point
        if(self.is_close(local_point)):
            #remove the point from the list
            self.points.pop(0)
        
        return v, w
        
if(UNIBOTICS):
    image_path='RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc/resources/images/mapgrannyannie.png'
else:
    image_path="map.png"
image_data=cv2.imread(image_path)
#create map grid
map = MapGrid(image_data, 45)
if(not UNIBOTICS):
    #draw the map
    new_image=map.draw_map(image_data)
    cv2.imshow("map",new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
#create the planner
robot_planner=RobotPlanner(map, map.real_to_grid_coords((-1,1.5)))
print("calculating plan")
for i in range(1000):
    robot_planner.step()
plan=robot_planner.actions
real_plan=[]
for point in plan:
    #translate plan from grid coords to real coords
    real_plan.append(map.grid_to_real_coords(point))
    print(round(real_plan[-1][0],2),round(real_plan[-1][1],2))
    time.sleep(1)
print("plan calculated")
#create the controller
robot_controller = RobotController(real_plan)

while True:
    if(UNIBOTICS):
        position=(HAL.getPose3d().x,HAL.getPose3d().y)
        angle=HAL.getPose3d().yaw
    else:
        position=(0,0)
        angle=0
    v,w=robot_controller.step(position,angle)
    print("current goal",robot_controller.points[0])
    print("current position",position)
    print(v,w)
    if(UNIBOTICS):
        HAL.setV(v)
        HAL.setW(w)        

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

27-sep: I implemented VFF (virtual force field) controller for the robot.
The robot moves to the points in the list using the VFF controller.
The main problem now is that the image has pixel coordinates, the map has grid coordinates and the robot has real coordinates.
I need to convert the coordinates between the three systems.
I don't know how to read the image data from unibotics, temporarily I will use a local image.
"""
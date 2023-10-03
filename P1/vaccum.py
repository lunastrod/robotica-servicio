
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
        self.image_data=self.image_preprocessing(image_data,18)
        self.width = self.image_data.shape[0]//grid_size
        self.height = self.image_data.shape[1]//grid_size
        self.grid=[["." for x in range(self.height)] for y in range(self.width)]
        for x in range(self.width):
            for y in range(self.height):
                #get the pixels of the cell
                cell_pixels = self.image_data[x*grid_size:(x+1)*grid_size, y*grid_size:(y+1)*grid_size]
                #check if the cell is an obstacle
                is_obstacle = np.any(cell_pixels == 0)
                #create the cell
                self.grid[x][y] = MapCell((x, y), is_obstacle)

    def image_preprocessing(self, image, robot_radius):
        # Perform dilation to enlarge the black obstacle zones
        inverted_image = cv2.bitwise_not(image)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*robot_radius+1, 2*robot_radius+1))
        enlarged_obstacles = cv2.dilate(inverted_image, kernel)
        enlarged_obstacles = cv2.bitwise_not(enlarged_obstacles)
        return enlarged_obstacles

    def __str__(self):
        text=f"Width: {self.width}, Height: {self.height}\n"
        for row in self.grid:
            for cell in row:
                text += f"{cell} "
            text += "\n"
        return text
    
    def draw_map(self):
        new_image = self.image_data
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

    def draw_plan(self, plan):
        new_image = self.image_data
        for i in range(self.height):
            for j in range(self.width):
                pixel_coords = self.grid_to_pixel_coords((j,i))
                if(self.grid[i][j].tags["obstacle"]):
                    cv2.rectangle(new_image, (pixel_coords[0]-self.grid_size//3, pixel_coords[1]-self.grid_size//3), (pixel_coords[0]+self.grid_size//3, pixel_coords[1]+self.grid_size//3), (0,0,127), -1)
                else:
                    cv2.rectangle(new_image, (pixel_coords[0]-self.grid_size//3, pixel_coords[1]-self.grid_size//3), (pixel_coords[0]+self.grid_size//3, pixel_coords[1]+self.grid_size//3), (127,0,0), -1)
        #paint the plan
        for i in range(len(plan)-1):
            cv2.line(new_image, self.real_to_pixel_coords(plan[i]), self.real_to_pixel_coords(plan[i+1]), (0,255,0), 2)

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
        return []
        
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
        x = position[0]*image_data.shape[0]/10
        y = position[1]*image_data.shape[1]/10
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
        x=self.grid_size*position[0]+self.grid_size/2
        y=self.grid_size*position[1]+self.grid_size/2
        #now we have pixel coords. convert to real coords translating and scaling
        x = x-580
        y = y-420
        #flip axis
        x=-x
        """
        temp = -y
        y = x
        x = temp
        """
        #scale
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
        x = round(x/self.grid_size)
        y = round(y/self.grid_size)
        return (x,y)
        


class RobotPlanner:
    def __init__(self, map, position):
        self.map = map
        self.position = (position[1],position[0])
        self.actions = []
        map.grid[position[1]][position[0]].edit_tag("robot", True)

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
        self.actions.append((self.position[1],self.position[0]))
        if(not UNIBOTICS):
            pass
            print(map)
            #time.sleep(0.2)

class RobotController:
    def __init__(self, plan):
        self.points = plan
        self.prev_error_distance = 0
        self.prev_error_angle = 0
    
    """
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
        maxv = 1.5
        maxw = 3
        kv = 3
        kw = 10

        angle = math.atan2(-force[1], force[0])
        #v=maxv/(1+kv*angle**2)-0.1685
        if(angle > -math.pi/12 and angle < math.pi/12):
            magnitude = math.sqrt(force[0]*force[0]+force[1]*force[1])
            v = min(maxv, kv*magnitude)
        else:
            v = 0
        #print("angle",angle)
        w=max(-maxw, min(maxw, kw*angle**3))
        return v, w
    """
    def pd_controller(self, start, goal, current_position, current_angle):
        # Constants for PD control
        Kp_distance = 0.5  # Proportional gain for distance control
        Kd_distance = 0.1  # Derivative gain for distance control
        Kp_angle = 0.2     # Proportional gain for angle control
        Kd_angle = 0.05    # Derivative gain for angle control

        # Calculate the distance and angle to the line
        distance = self.calculate_distance(current_position, start, goal)
        angle = self.calculate_angle(current_position, start, goal, current_angle)

        # Calculate the error and its rate of change for distance control
        error_distance = distance
        delta_error_distance = error_distance - self.prev_error_distance

        # Calculate the error and its rate of change for angle control
        error_angle = angle
        delta_error_angle = error_angle - self.prev_error_angle

        # PD control to determine linear and angular velocities
        v = Kp_distance * error_distance + Kd_distance * delta_error_distance
        w = Kp_angle * error_angle + Kd_angle * delta_error_angle

        # Update the previous errors for the next iteration
        self.prev_error_distance = error_distance
        self.prev_error_angle = error_angle

        return v, w

    def calculate_distance(self, current_position, start, goal):
        # Euclidean distance between the current position and the line formed by start and goal
        x, y = current_position
        x1, y1 = start
        x2, y2 = goal

        distance = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1)**2 + (x2 - x1)**2)

        return distance

    def calculate_angle(self, current_position, start, goal, current_angle):
        # Calculate the angle between the current position and the line formed by start and goal
        x, y = current_position
        x1, y1 = start
        x2, y2 = goal

        angle_to_line = math.atan2(y2 - y1, x2 - x1)
        angle_to_position = math.atan2(y - y1, x - x1)

        # Calculate the signed angle difference
        angle_diff = angle_to_position - angle_to_line

        # Convert angle to the range [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        print("angle_diff",angle_diff)

        return angle_diff - current_angle
    
    def is_close(self, goal):
        threshold = 0.05
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
        next_point = self.points[1]
        #print("local_point",local_point)
        """
        #get the goal force
        goal_force = self.VFF_controller(local_point)
        #get the linear and angular velocity
        v, w = self.force_to_vw(goal_force)
        """
        v, w = self.pd_controller(point, next_point, position, angle)
        #check if the robot is close to the point
        if(self.is_close(self.local_coords(point, position, angle))):
            #remove the point from the list
            self.points.pop(0)
        
        return v, w
        
if(UNIBOTICS):
    image_path='RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc/resources/images/mapgrannyannie.png'
else:
    image_path="map.png"
image_data=cv2.imread(image_path)
#create map grid
map = MapGrid(image_data, 20)
if(not UNIBOTICS):
    #draw the map
    new_image=map.draw_map()
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
print("plan calculated")
if(not UNIBOTICS):
    new_image=map.draw_plan(real_plan)
    cv2.imshow("plan",new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
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

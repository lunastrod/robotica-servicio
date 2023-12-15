from HAL import HAL
from GUI import GUI
from ompl import base, geometric

class RobotMotionPlanner:
    def __init__(self):
        self.hal = HAL()
        self.gui = GUI()

        # Warehouse information
        self.warehouse_length = 20.62
        self.warehouse_width = 13.6
        self.shelf_coordinates = [
            (3.728, 0.579),
            (3.728, -1.242),
            (3.728, -3.039),
            (3.728, -4.827),
            (3.728, -6.781),
            (3.728, -8.665)
        ]
        self.start=(0,0)
        self.si=base.SpaceInformation()

        map_file = "P4/map.pgm"


    def configure_state_space(self):
        space = base.RealVectorStateSpace(2) #coordinates of the robot (x,y)
        bounds = base.RealVectorBounds(2) #bounds of the warehouse (2d), defined by 2 opposite corners
        bounds.setLow([0, -self.warehouse_width/2])  
        bounds.setHigh([self.warehouse_length, self.warehouse_width/2])
        space.setBounds(bounds)
        self.si.setStateSpace(space)

    def configure_problem(self, start_coords, goal_coords):
        start = base.State(self.si)
        goal = base.State(self.si)

        # Set start and goal states
        start[0] = start_coords[0]
        start[1] = start_coords[1]
        goal[0] = goal_coords[0]
        goal[1] = goal_coords[1]

        pdef = base.ProblemDefinition(self.si)
        pdef.setStartAndGoalStates(start, goal)
        pdef.setSolutionPath(base.PathPtr(self.si))
        return pdef

    def plan_with_rrt(self):
        planner = geometric.RRT(self.si)
        self.si.getMotionPlanner(planner)

        # Set the maximum planning time
        planner.setRange(5.0)

        # Set optimization objective for shortest path
        obj = base.PathLengthOptimizationObjective(self.si)
        planner.setOptimizationObjective(obj)

        # Solve the problem
        solved = planner.solve(10.0)

        if solved:
            path = planner.getSolutionPath()
            path.interpolate()

            # Extract waypoints from the path
            waypoints = []
            for state in path.getStates():
                waypoints.append((state[0], state[1]))

            return waypoints
        else:
            print("No solution found.")
            return None

    def execute_path(self, waypoints):
        for waypoint in waypoints:
            # Move the robot to the waypoint
            x, y = waypoint
            self.hal.setV(1.0)  # Set linear speed
            self.hal.setW(0.0)  # Set angular speed

            # Update the robot's position
            self.hal.getPose3d()

            # Visualize the path on the map
            self.gui.showPath([waypoint])

        # Stop the robot after reaching the goal
        self.hal.setV(0.0)
        self.hal.setW(0.0)

if __name__ == "__main__":
    planner = RobotMotionPlanner()

    # Set up the planning context
    planner.configure_state_space()
    problem_def = planner.configure_problem(planner.start, planner.shelf_coordinates[0])

    # Plan with RRT
    waypoints = planner.plan_with_rrt()

    if waypoints:
        # Execute the planned path
        planner.execute_path(waypoints)
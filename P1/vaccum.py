"""
implement the logic of a navigation algorithm for an autonomous vacuum cleaner by making use of the location of the robot.
The robot is equipped with a map and knows it’s current location in it.
The main objective will be to cover the largest area of ​​a house using the programmed algorithm

Our solution will be a complete coverage BSA (backtracking spiral algorithm).
This algorithm divides the map into a grid.
"""
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

    def __str__(self):
        return f"Position: {self.position}, Tags: {self.tags}"
    
    def __repr__(self):
        return f"Position: {self.position}, Tags: {self.tags}"

    def add_tag(self, tag):
        self.tags[tag] = True

    def remove_tag(self, tag):
        if tag in self.tags:
            del self.tags[tag]


class MapGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[MapCell((x, y), False) for x in range(width)] for y in range(height)]
    def __str__(self):
        return f"Width: {self.width}, Height: {self.height}, Grid: {self.grid}"
    

if __name__ == "__main__":
    map=MapGrid(10,10)
    print(map)

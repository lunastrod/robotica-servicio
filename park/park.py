UNIBOTICS=True
if UNIBOTICS:
    from GUI import GUI
    from HAL import HAL

states = [
    #move to the right until close to the cars
    {"ID":0,    "NAME":"GETTING_CLOSER",   "VW":(2,-0.2), "CONDITION":lambda sensor_data,angle: sensor_data["right"]<1 or angle<-3.14/5},
    #turn left until parallel to the cars
    {"ID":1,    "NAME":"CORRECTING",       "VW":(1,0.3),  "CONDITION":lambda sensor_data,angle: angle>=0},
    #move fordwards until you find a parking spot
    {"ID":2,    "NAME":"SEARCHING",        "VW":(1,0),    "CONDITION":lambda sensor_data,angle: sensor_data["positioning_point"]>5},
    #move fordwards until in the right position
    {"ID":3,    "NAME":"POSITIONING",      "VW":(1,0),    "CONDITION":lambda sensor_data,angle: sensor_data["positioning_point"]<5},
    #
    {"ID":4,    "NAME":"BACKWARDS",        "VW":(-1,0.2), "CONDITION":lambda sensor_data,angle: angle>3.14/4},
    #
    {"ID":5,    "NAME":"CORRECTING",      "VW":(-1,-0.2),"CONDITION":lambda sensor_data,angle: angle<0},
    #STOP
    {"ID":6,    "NAME":"STOP",             "VW":(0,0),    "CONDITION":lambda sensor_data,angle: False},
]

class RobotController:
    """
    Class to control the robot
    """
    def __init__(self):
        self.state=states[0]
    def sensor_processing(self,raw_sensor_data):
        """
        process the raw sensor data and return a dictionary with the relevant data
        the raw sensor data is a list of 3 elements (front, right, back)
        each element is a list of 180 values (one for each degree)
        the values are the distance in meters
        """
        RIGHT_END=45
        LEFT_START=136
        sensor_data={}
        sensor_data["right"]=min(raw_sensor_data[1][RIGHT_END:LEFT_START])
        sensor_data["front"]=min(raw_sensor_data[0])
        sensor_data["positioning_point"]=raw_sensor_data[2][179]
        sensor_data["start_correcting_point"]=raw_sensor_data[1][100]
        return sensor_data
    
    def step(self,sensor_data,angle):
        """
        execute one step of the state machine
        """
        if(self.state["CONDITION"](sensor_data,angle)):
            self.state=states[self.state["ID"]+1]
        return self.state["VW"]


controller=RobotController()
while(True):
    if(UNIBOTICS):
        raw_sensor_data=(HAL.getFrontLaserData().values,HAL.getRightLaserData().values,HAL.getBackLaserData().values)
        #if length if any of the sensor data, continue
        if(len(raw_sensor_data[0])!=0 and len(raw_sensor_data[1])!=0 and len(raw_sensor_data[2])!=0):
            sensor_data=controller.sensor_processing(raw_sensor_data)
            v,w=controller.step(sensor_data,HAL.getPose3d().yaw)
            HAL.setV(v)
            HAL.setW(w)
            print("state: "+controller.state["NAME"])
            print("v: "+str(v)+" w: "+str(w))
            print("sensor_data: "+str(sensor_data))
            position=HAL.getPose3d().x,HAL.getPose3d().y,HAL.getPose3d().yaw
            print("position: "+str(position))

class States:
    SEARCHING=0
    POSITIONING=1
    PARKING1=2
    PARKING2=3
    PARKING3=4
    PARKING4=5
    PARKED=6
    @staticmethod
    def to_str(state):
        if state==States.SEARCHING:
            return "SEARCHING"
        elif state==States.POSITIONING:
            return "POSITIONING"
        elif state==States.PARKING1:
            return "PARKING1"
        elif state==States.PARKING2:
            return "PARKING2"
        elif state==States.PARKING3:
            return "PARKING3"
        elif state==States.PARKING4:
            return "PARKING4"
        elif state==States.PARKED:
            return "PARKED"
        else:
            return "UNKNOWN"

class RobotController:
    """
    Class to control the robot
    """
    def __init__(self):
        self.state=States.SEARCHING

    def sensor_processing(self,raw_sensor_data):
        """
        process the raw sensor data and return a dictionary with the relevant data
        the raw sensor data is a list of 3 elements (front, right, back)
        each element is a list of 180 values (one for each degree)
        the values are the distance in meters
        """
        RIGHT_END=45
        LEFT_START=136
        sensor_data={}
        sensor_data["right"]=min(raw_sensor_data[1][RIGHT_END:LEFT_START])
        sensor_data["front"]=min(raw_sensor_data[0])
        sensor_data["positioning_point"]=raw_sensor_data[2][179]
        sensor_data["start_correcting_point"]=raw_sensor_data[1][100]
        return sensor_data


    def step(self,sensor_data,angle):
        """
        execute one step of the state machine
        """
        v,w=0,0
        if self.state==States.SEARCHING:
            v,w=self.searching(sensor_data,angle)
        elif self.state==States.POSITIONING:
            v,w=self.positioning(sensor_data,angle)
        elif self.state==States.PARKING1:
            v,w=self.parking1(sensor_data,angle)
        elif self.state==States.PARKING2:
            v,w=self.parking2(sensor_data,angle)
        elif self.state==States.PARKING3:
            v,w=self.parking3(sensor_data,angle)
        elif self.state==States.PARKING4:
            v,w=self.parking4(sensor_data,angle)
        elif self.state==States.PARKED:
            v,w=0,0
        return v,w
    def searching(self,sensor_data,angle):
        """
        search for a parking spot
        move fordward until right sensor doesnt detect a car
        """
        if(sensor_data["right"]>5):
            self.state=States.POSITIONING
            return 0,0
        return 2,0
    def positioning(self,sensor_data,angle):
        """
        position the robot in front of the parking spot
        move forward until the back sensor detects a car
        """
        if(sensor_data["positioning_point"]<5):
            self.state=States.PARKING1
            return 0,0
        return 1,0
    def parking1(self,sensor_data,angle):
        """
        first turn
        turn left until 45 degrees
        """
        if(angle>3.14/4):
            self.state=States.PARKING2
            return 0,0
        return -0.5,0.10
    def parking2(self,sensor_data,angle):
        """
        move backwards
        """
        if(sensor_data["start_correcting_point"]>5):
            self.state=States.PARKING3
            return 0,0
        return -0.5,0
    def parking3(self,sensor_data,angle):
        """
        second turn
        turn right until 0 degrees
        """
        if(angle<0.01):
            self.state=States.PARKING4
            return 0,0
        return -0.5,-0.13
    def parking4(self,sensor_data,angle):
        """
        move forward
        move forward until the front sensor detects a car closer
        """
        if(sensor_data["front"]<1):
            self.state=States.PARKED
            return 0,0
        return 0.5,0

    

UNIBOTICS=True
if UNIBOTICS:
    from GUI import GUI
    from HAL import HAL
class States:
    SEARCHING=0
    POSITIONING=1
    PARKING1=2
    PARKING2=3
    PARKING3=4
    PARKING4=5
    PARKED=6
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
        the right of a sensor are the values from 0 to 45 degrees
        the front of a sensor are the values from 46 to 135 degrees
        the left of a sensor are the values from 136 to 179 degrees
        we will use the minimum value of each sensor
        """
        RIGHT_END=45
        LEFT_START=136
        sensor_data={}
        sensor_data["front_left"]=min(raw_sensor_data[0][LEFT_START:])
        sensor_data["front"]=min(raw_sensor_data[0][RIGHT_END:LEFT_START])
        sensor_data["front_right"]=min(raw_sensor_data[0][:RIGHT_END])
        sensor_data["right"]=min(raw_sensor_data[1][RIGHT_END:LEFT_START])
        sensor_data["back_right"]=min(raw_sensor_data[2][:RIGHT_END])
        sensor_data["back"]=min(raw_sensor_data[2][RIGHT_END:LEFT_START])
        sensor_data["back_left"]=min(raw_sensor_data[2][LEFT_START:])
        
        return sensor_data


    def step(self,sensor_data):
        """
        execute one step of the state machine
        """
        v,w=0,0
        if self.state==States.SEARCHING:
            v,w=self.searching(sensor_data)
        elif self.state==States.POSITIONING:
            v,w=self.positioning(sensor_data)
        elif self.state==States.PARKING1:
            v,w=self.parking1(sensor_data)
        elif self.state==States.PARKING2:
            v,w=self.parking2(sensor_data)
        elif self.state==States.PARKING3:
            v,w=self.parking3(sensor_data)
        elif self.state==States.PARKING4:
            v,w=self.parking4(sensor_data)
        elif self.state==States.PARKED:
            v,w=0,0
        return v,w
    def searching(self,sensor_data):
        """
        search for a parking spot
        move fordward until right sensor doesnt detect a car
        """
        if(sensor_data["right"]>4):
            self.state=States.POSITIONING
            return 0,0
        return 0.5,0
    def positioning(self,sensor_data):
        """
        position the robot in front of the parking spot
        move forward until the back sensor detects a car
        """
        if(sensor_data["back_left"]<4):
            self.state=States.PARKING1
            return 0,0
        return 0.5,0
    def parking1(self,sensor_data):
        """
        first turn
        turn left until the right sensor detects the gap between the cars
        """
        if(sensor_data["right"]>3):
            self.state=States.PARKING2
            return 0,0
        return -0.5,0.5
    def parking2(self,sensor_data):
        """
        move backwards
        move backwards until the back sensor detects a car
        """
        if(sensor_data["back_right"]<1):
            self.state=States.PARKING3
            return 0,0
        return -0.5,0
    def parking3(self,sensor_data):
        """
        second turn
        turn right until the front sensor detects a car
        """
        if(sensor_data["front"]<4):
            self.state=States.PARKING4
            return 0,0
        return -0.5,-0.5
    def parking4(self,sensor_data):
        """
        move forward
        move forward until the front sensor detects a car closer
        """
        if(sensor_data["front"]<1):
            self.state=States.PARKED
            return 0,0
        return 0.5,0
    
    
controller=RobotController()
while(True):
    if(UNIBOTICS):
        raw_sensor_data=(HAL.getFrontLaserData(),HAL.getRightLaserData(),HAL.getBackLaserData())
        sensor_data=controller.sensor_processing(raw_sensor_data)
        v,w=controller.step(sensor_data)
        HAL.set_velocity(v,w)
        print("state: "+States.to_str(controller.state))
        print("v: "+str(v)+" w: "+str(w))
        print("sensor_data: "+str(sensor_data))
    else:
        print("UNIBOTICS is not defined")
        break
UNIBOTICS=True
if UNIBOTICS:
    from GUI import GUI
    from HAL import HAL
import numpy as np

states = [
    #turn right until close to the cars
    {"ID":0,    "NAME":"GETTING_CLOSER",   "VW":(2,-0.2), "CONDITION":lambda sensor_data,angle: sensor_data["right"]<1 or angle<-3.14/5},
    #turn left until parallel to the cars
    {"ID":1,    "NAME":"CORRECTING",       "VW":(2,0.3),  "CONDITION":lambda sensor_data,angle: angle>=-0.04},
    #move forwards until you find a parking spot
    {"ID":2,    "NAME":"SEARCHING",        "VW":(2,0),    "CONDITION":lambda sensor_data,angle: sensor_data["right"]>5 and sensor_data["positioning_point"]>5},
    #move forwards until your back wheel is close to the end of the car
    {"ID":3,    "NAME":"POSITIONING",      "VW":(1,0),    "CONDITION":lambda sensor_data,angle: sensor_data["positioning_point"]<5},
    #move backwards turning right until your angle is 36 degrees
    {"ID":4,    "NAME":"BACKWARDS",        "VW":(-1,2),   "CONDITION":lambda sensor_data,angle: angle>3.14/5},
    #move backwards turning left until you're about to bump the back car
    {"ID":5,    "NAME":"CORRECTING",       "VW":(-1,-2),  "CONDITION":lambda sensor_data,angle: angle<0 or sensor_data["back"]<0.6},
    #move forwards turning left until you're about to bump the front car
    {"ID":6,    "NAME":"FORWARDS",        "VW":(1,-2),   "CONDITION":lambda sensor_data,angle: angle<0 or sensor_data["front"]<0.6},
    #STOP
    {"ID":7,    "NAME":"STOP",             "VW":(0,0),    "CONDITION":lambda sensor_data,angle: False},
]

def calculate_main_direction(lidar_data):
    # Agregar una columna de unos para el término independiente
    X = np.column_stack((np.ones_like(lidar_data[:, 0]), lidar_data[:, 0]))
    # Calcular los coeficientes de regresión usando la pseudo-inversa de X
    coefficients = np.linalg.pinv(X) @ lidar_data[:, 1]
    print('Coeficientes de regresión: {}'.format(coefficients))
    return coefficients

def detect_segments(lidar_data, threshold=0.1):
    # Find changes in slope
    slopes = np.gradient(lidar_data)
    
    # Identify segment boundaries based on slope changes
    segment_boundaries = np.where(np.abs(slopes) > threshold)[0]

    # Split the lidar data into segments
    segments = np.split(lidar_data, segment_boundaries + 1)
    
    print(segments)

    return segments

def polar_to_cartesian(r, theta):
    return r * np.cos(theta), r * np.sin(theta)

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
        sensor_data["back"]=min(raw_sensor_data[2])
        sensor_data["positioning_point"]=raw_sensor_data[2][165]
        #sensor_data["start_correcting_point"]=raw_sensor_data[1][100]
        detect_segments(raw_sensor_data[1])
        
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

#==============================================================================

import matplotlib.pyplot as plt

inf = float('inf')

data=[inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,3.2 ,3.1 ,3.0 ,2.8 ,2.7 ,2.6 ,2.5 ,2.5 ,2.5 ,2.4 ,2.4 ,2.3 ,2.3 ,2.3 ,2.3 ,2.2 ,2.2 ,2.2 ,2.1 ,2.1 ,2.1 ,2.1 ,2.1 ,2.1 ,2.0 ,2.0 ,2.0 ,2.0 ,2.0 ,2.0 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.9 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.7 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,2.0 ,1.9 ,1.9 ,1.9 ,2.0 ,2.0 ,2.0 ,2.1 ,2.1 ,2.1 ,2.1 ,2.2 ,2.2 ,2.2 ,2.3 ,2.3 ,2.4 ,2.4 ,2.5 ,2.5 ,2.6 ,2.6 ,inf ,inf ,inf ,4.6 ,4.6 ,4.4 ,4.4 ,4.3 ,4.2 ,4.2 ,4.1 ,4.1 ,4.0 ,4.0 ,4.0 ,4.0 ,4.0 ,3.9 ,3.9 ,3.9 ,3.9 ,4.0 ,4.1 ,4.3 ,4.3 ,4.4 ,4.7 ,5.3 ,5.7 ,6.1 ,6.7 ,7.2 ,10.0 ,10.0 ,10.1 ,10.4 ,12.8 ,21.6 ,21.6 ,21.7 ,27.6 ,inf ,inf]

lidar_data = []
for i in range(len(data)):
    if(data[i]==inf):
        #data[i]=5
        continue
    if(i<0 or i>180):
        continue
    #print(i/180*np.pi)
    lidar_data.append(polar_to_cartesian(data[i],i/180*np.pi))

#print(lidar_data)
lidar_data=np.array(lidar_data)


coefficients=calculate_main_direction(lidar_data)
# Visualiza los resultados
plt.scatter(lidar_data[:, 0], lidar_data[:, 1], label='Datos Lidar', color='blue', marker='.')
#generate values to draw the line
x_values = np.linspace(-5, 5, 100)
y_values = coefficients[0] + coefficients[1] * x_values
#plot the line
# Dibuja el punto (0, 0)
plt.scatter(0, 0, color='green', marker='o', label='Punto (0, 0)')
plt.plot(x_values, y_values, color='red', linewidth=2, label='Ajuste Lineal')

plt.xlim(left=-5, right=5)
plt.ylim(bottom=-1, top=7)
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()
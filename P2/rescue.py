UNIBOTICS=0
if(UNIBOTICS):
    from GUI import GUI
    from HAL import HAL
#https://omes-va.com/deteccion-de-rostros-con-haar-cascades-python-opencv/
import cv2
from math import radians, cos, sin, degrees
import time
import numpy as np

faceClassif = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def rotate_image(img, angle):
    size_reverse = np.array(img.shape[1::-1]) # swap x with y
    M = cv2.getRotationMatrix2D(tuple(size_reverse / 2.), angle, 1.)
    MM = np.absolute(M[:,:2])
    size_new = MM @ size_reverse
    M[:,-1] += (size_new - size_reverse) / 2.
    return cv2.warpAffine(img, M, tuple(size_new.astype(int)))

def detect_faces(image):
    image_out=image.copy()
    image_processing=image.copy()
    image_processing=cv2.cvtColor(image_out, cv2.COLOR_BGR2GRAY)
    is_face=False
    ROTATIONS=4
    rot=[360/ROTATIONS]*ROTATIONS
    for angle in rot:
        image_processing = rotate_image(image_processing, angle)
        image_out = rotate_image(image_out, angle)
        faces=faceClassif.detectMultiScale(image_processing,scaleFactor=1.1,minNeighbors=5,minSize=(10,10),maxSize=(300,300))
        if(len(faces)>0):
            is_face=True
        for (x,y,w,h) in faces:
            cv2.rectangle(image_out,(x,y),(x+w,y+h),(0,255,0),2)
    return image_out,is_face




def gps_to_cartesian(origin, destination):
    # Radius of the Earth in meters
    R = 6371000.0
    # Convert latitude and longitude from degrees to radians
    origin_lat, origin_lon = map(radians, origin)
    dest_lat, dest_lon = map(radians, destination)
    # Calculate differences in coordinates
    dlat = dest_lat - origin_lat
    dlon = dest_lon - origin_lon
    # Linear conversion (for short distances)
    x = R * dlon * cos(origin_lat)
    y = R * dlat
    return x, y

def cartesian_to_gps(origin, destination):
    # Radius of the Earth in meters
    R = 6371000.0
    # Convert latitude and longitude from degrees to radians
    origin_lat, origin_lon = map(radians, origin)
    # Linear conversion (for short distances)
    lat = destination[1] / R + origin_lat
    lon = destination[0] / (R * cos(origin_lat)) + origin_lon
    # Convert back to degrees
    lat, lon = map(degrees, [lat, lon])
    return lat, lon

def cartesian_to_local(point):
    return -point[0],-point[1]

def local_to_cartesian(point):
    return -point[0],-point[1]



class SearchPattern:
    def __init__(self):
        self.vx=1
        self.w=1
        self.time_straight=0
        self.dtime_straight=0.007
        self.time_turn=0.1
        self.progress=250
    def step(self):
        self.time_straight+=self.dtime_straight
        self.progress-=1
        if(self.progress<=0):
            return False,False,False,False,False
        return self.time_straight,self.vx,self.time_turn,self.w,self.progress
    
#CONSTANTS:
SAFETY_BOAT_POSITION=(40.27978611111111,-3.817161111111111)
SURVIVORS_POSITION=(40.280055555555556,-3.817638888888889)
LOCAL_SURVIVORS_POSITION=cartesian_to_local(gps_to_cartesian(SAFETY_BOAT_POSITION,SURVIVORS_POSITION))
print("LOCAL_SURVIVORS_POSITION",LOCAL_SURVIVORS_POSITION)
HAL.takeoff(3)
HAL.set_cmd_pos(LOCAL_SURVIVORS_POSITION[0],LOCAL_SURVIVORS_POSITION[1], 3, 0)
spiral_control=SearchPattern()
time.sleep(10)
survivors=[]
while True:
    image=HAL.get_ventral_image()
    GUI.showLeftImage(image)
    image,faces=detect_faces(image)
    print("faces:",faces)
    GUI.showImage(image)
    if(faces):
        p=HAL.get_position()
        p=cartesian_to_gps(SAFETY_BOAT_POSITION,local_to_cartesian((p[0],p[1])))
        #if too similar to any other survivor, then ignore
        for s in survivors:
            if(abs(p[0]-s[0])<0.00001 + abs(p[1]-s[1])<0.00001):
                break
        else:
            survivors.append(p)
        

    ts,vx,tt,w,progress=spiral_control.step()
    print("progress:",progress)
    if(ts==False):
        print("survivors:",survivors)
        HAL.set_cmd_pos(0, 0, 3, 0)
        time.sleep(10)
        HAL.land()
    HAL.set_cmd_mix(vx, 0, 3, 0)
    time.sleep(ts)
    HAL.set_cmd_mix(vx, 0, 3, w)
    time.sleep(tt)
    print(ts,"s",vx,"m/s",tt,"s",w,"rad/s")

    

#spiral search pattern from origin until size
"""
import turtle
import math
t = turtle.Pen()
t.speed(0)
t.width(50)
t.color("blue")
radius = 1.25
for i in range(15,1000):
    angle =3.141592/20*i
    x = math.cos(angle)*i*radius
    y = math.sin(angle)*i*radius
    t.goto(x,y)
turtle.getscreen()._root.mainloop()
"""
"""
class SearchPattern:
    def __init__(self,origin=(0,0),radius=0.01,size=20,step_size=10):
        self.origin=origin
        self.radius=radius
        self.size=size
        self.step_size=step_size
        self.progress=0
        self.current=origin
    def step(self):
        angle =3.141592/self.size*self.progress
        x = cos(angle)*self.progress*self.radius
        y = sin(angle)*self.progress*self.radius
        self.current=(x+self.origin[0],y+self.origin[1])
        self.progress+=self.step_size
        return self.current
    def spiral(self,position,threshold=0.1):
        #if position is similar to self.current, then step and return self.current
        #else return self.current
        if(abs(position[0]-self.current[0])<threshold and abs(position[1]-self.current[1])<threshold):
            return self.step()
        else:
            return self.current
"""
"""
vx=1
w=1
time_straight=0
dtime_straight=0.007
time_turn=0.1
HAL.takeoff(3)
HAL.set_cmd_pos(40, -30, 3, 0)
time.sleep(10)
while True:
    HAL.set_cmd_mix(vx, 0, 3, 0)
    time.sleep(time_straight)
    HAL.set_cmd_mix(vx, 0, 3, w)
    time.sleep(time_turn)
    time_straight+=dtime_straight
    print(time_straight,"s",vx,"m/s",time_turn,"s",w,"rad/s")
    
    
    GUI.showImage(HAL.get_ventral_image())
    GUI.showLeftImage(HAL.get_frontal_image())
"""

"""
#INITIALIZE:
spiral_control=SearchPattern(origin=LOCAL_SURVIVORS_POSITION)
while True:
    image=HAL.get_ventral_image()
    GUI.showLeftImage(image)
    image,faces=detect_faces(image)
    print("faces:",faces)

    if(UNIBOTICS):
        GUI.showImage(image)
        print(HAL.get_landed_state())
        print(HAL.get_position())
        if(HAL.get_landed_state()<=1):
            print("takeoff")
            HAL.takeoff(3)
        else:
            x,y,z=HAL.get_position()
            x,y=spiral_control.spiral((x,y))
            HAL.set_cmd_pos(x, y, 3, 0)
    else:
        cv2.imshow("image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        time.sleep(1)

"""

"""
import turtle
import math
t = turtle.Pen()
t.speed(0)
t.width(50)
t.color("blue")
radius = 1.25
for i in range(15,1000):
    angle =3.141592/20*i
    x = math.cos(angle)*i*radius
    y = math.sin(angle)*i*radius
    t.goto(x,y)
turtle.getscreen()._root.mainloop()
"""
"""
class SearchPattern:
    def __init__(self,origin=(0,0),radius=0.01,size=20,step_size=10):
        self.origin=origin
        self.radius=radius
        self.size=size
        self.step_size=step_size
        self.progress=0
        self.current=origin
    def step(self):
        angle =3.141592/self.size*self.progress
        x = cos(angle)*self.progress*self.radius
        y = sin(angle)*self.progress*self.radius
        self.current=(x+self.origin[0],y+self.origin[1])
        self.progress+=self.step_size
        return self.current
    def spiral(self,position,threshold=0.1):
        #if position is similar to self.current, then step and return self.current
        #else return self.current
        if(abs(position[0]-self.current[0])<threshold and abs(position[1]-self.current[1])<threshold):
            return self.step()
        else:
            return self.current
"""


"""
from GUI import GUI
from HAL import HAL
print("start")
while True:
    GUI.showImage(HAL.get_ventral_image())
    print(HAL.get_landed_state())
    print(HAL.get_position())
    if(HAL.get_landed_state()<=1):#take off if landed
      print("takeoff")
      HAL.takeoff(3)
    else:#move if flying
      HAL.set_cmd_pos(40, -30, 3, 0)
"""
"""
t=time.time()
for i in range(20):
    print("Faces detected",detect_faces(image))
t=time.time()-t
print("Time ",t,"s")

origin=(40.27978611111111,-3.817161111111111)
destin=(40.280055555555556,-3.817638888888889)
#      (40.27951631462933, -3.8166895810188857)

print(cartesian_to_local(gps_to_cartesian(origin,destin)))
print(cartesian_to_local(gps_to_cartesian(origin,cartesian_to_gps(origin,(-40,30)))))
"""


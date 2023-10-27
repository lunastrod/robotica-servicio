UNIBOTICS=0
if(UNIBOTICS):
    from GUI import GUI
    from HAL import HAL
#https://omes-va.com/deteccion-de-rostros-con-haar-cascades-python-opencv/
import cv2
import time
import math

image = cv2.imread('f3.jpeg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
faceClassif = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def detect_faces(image):
    """
    for angle in range(0, 360, 15):
    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
    """
    faces = faceClassif.detectMultiScale(image,scaleFactor=1.1,minNeighbors=5,minSize=(1,1),maxSize=(500,500))
    """
    for (x,y,w,h) in faces:
        #cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.imshow("Rotated Image", rotated_image)
        cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #time.sleep(1)
    """
    return len(faces) > 0

from math import radians, cos, degrees


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
from GUI import GUI
from HAL import HAL
# Enter sequential code!
while True:
    GUI.showImage(HAL.get_ventral_image())
    print(HAL.get_landed_state(),HAL.get_position())
    if(HAL.get_landed_state()<=1):#take off if landed
      print("takeoff")
      HAL.takeoff(5)
    else:#move if flying
      print("moving")
      HAL.set_cmd_pos(40, -30, 5, 0)
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


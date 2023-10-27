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

#funtion to calculate the distance (x,y) between two gps points
def distance(lat1,lon1,lat2,lon2):
    R = 6373.0
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = (math.sin(dlat/2))**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2))**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c
    print(distance)
    


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

t=time.time()
for i in range(20):
    print("Faces detected",detect_faces(image))
t=time.time()-t
print("Time ",t,"s")

print(distance(40.27978611111111,-3.817161111111111,40.280055555555556,-3.817638888888889))


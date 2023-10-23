UNIBOTICS=0
if(UNIBOTICS):
    from GUI import GUI
    from HAL import HAL
#https://omes-va.com/deteccion-de-rostros-con-haar-cascades-python-opencv/
import cv2

image = cv2.imread('faces.png')
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
faceClassif = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def detect_faces(image):
    faces = faceClassif.detectMultiScale(image,scaleFactor=1.1,minNeighbors=5,minSize=(10,10),maxSize=(500,500))
    for (x,y,w,h) in faces:
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
    print("Faces detected: ", len(faces))

#faces = faceClassif.detectMultiScale(gray,scaleFactor=1.01,minNeighbors=5,minSize=(10,10),maxSize=(500,500))
#rotate the image before detecting the faces, rotate 90 degrees
detect_faces(image)
image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
detect_faces(image)
image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
detect_faces(image)
image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
detect_faces(image)
image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)


cv2.imshow('image',image)
cv2.waitKey(0)
cv2.destroyAllWindows()
# python -m idlelib
import time
print("import "+str(time.time()))
import cv2
import face_recognition
print("coucou "+str(time.time()))
camera = cv2.VideoCapture(0)
while cv2.waitKey(100) == -1:
    (grabbed, frame) = camera.read()
    cv2.imshow("Camera", frame)
rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
print("detection "+str(time.time()))
boxes = face_recognition.face_locations(rgb, model='hog')
for (top, right, bottom, left) in boxes:
    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
cv2.imshow("Boxes", frame)
print("alu "+str(time.time()))
cv2.waitKey() 
camera.release()

import time
print("import "+str(time.time()))
import cv2
import face_recognition
import RPi.GPIO as GPIO
print("coucou "+str(time.time()))

class cou:
    centreX = 6.6
    fx = centreX
    centreY = 4
    fy = centreY
    quart = 2
    pateX = 18 # 13
    pateY = 12
    actif = False
    
    def __init__(o):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(o.pateX, GPIO.OUT)
        GPIO.setup(o.pateY, GPIO.OUT)
        o.pwmH = GPIO.PWM(o.pateX, 50)
        o.pwmV = GPIO.PWM(o.pateY, 50)
        o.pwmH.start(o.centreX)
        o.pwmV.start(o.centreY)
        o.fx = o.centreX
        o.fy = o.centreY
        actif= True

    def bye(o):
        o.pwmH.stop()
        o.pwmV.stop()
        GPIO.cleanup()

    def pause(o):
        o.pwmH.stop()
        o.pwmV.stop()
        o.actif= False

    def reprendre(o):
        if not(o.actif):
            o.pwmH.start(o.fx)
            o.pwmV.start(o.fy)
            o.actif= True
            
    def centre(o):
        o.fx = o.centreX
        o.fy = o.centreY
        o.position()
        
    def position(o):
        o.pwmH.ChangeDutyCycle(o.fx)
        o.pwmV.ChangeDutyCycle(o.fy)
        return o.fx,o.fy

    def gauche(o):
        o.fx = o.centreX+o.quart
        o.position()
    def droite(o):
        o.fx = o.centreX-o.quart
        o.position()
    def haut(o):
        o.fy = o.centreY-o.quart
        o.position()
    def bas(o):
        o.fy = o.centreY+o.quart
        o.position()
    def monte(o, dy):
        o.fy += quart * (-dy)
        o.position()
    def tourne(o, dx):
        o.fx += quart * dx
        o.position()
        

class oeuil:
    def __init__(o, perif = 0):
        o.camera = cv2.VideoCapture(perif)

    def camera(o):
        return o.camera

    def boxvisage(o):
        (grabbed, frame) = o.camera.read()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        boxes = face_recognition.face_locations(rgb, model='hog')
        for (top, right, bottom, left) in boxes:
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        return boxes, frame

    def aff(o, frame, wait = True):
        cv2.imshow("Oeuil", frame)
        if wait:
            cv2.waitKey()

    def cible_grand(o, boxes_frame):
        (boxes, frame) = boxes_frame
        grand = max([x2-x1+y2-y1 for (y1,x2,y2,x1) in boxes])
        boxes_par_taille = { x2-x1+y2-y1 : (y1,x2,y2,x1) for (y1,x2,y2,x1) in boxes }
        (top, right, bottom, left) = boxes_par_taille[grand]
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        (h, l, _) = frame.shape
        # vecteur partant du centre de l'ecran dans la direction du centre du rectangle ciblé
        return (((top + bottom - h)/2/h), ((left + right - l)/2/l))
        
        
    def cible_centre(o, boxes_frame):
        (boxes, frame) = boxes_frame
        

def test():
    t=cou()
    t.centre()
    time.sleep(1)
    t.gauche()
    time.sleep(1)
    t.droite()
    time.sleep(1)
    t.centre()
    t.bye()

if __name__ == "__main__":
    test()

import time
print("import "+str(time.time()))
import pigpio
import numpy as np
import cv2
import face_recognition
print("coucou "+str(time.time()))

class cou:
    centreX = 19
    fx = centreX
    minX = 9
    maxX = 29
    centreY = 12
    fy = centreY
    minY = 9
    maxY = 22
    quart = 7
    pateX = 13
    pateY = 12
    actif = False
    
    def __init__(o):
        o.pi = pigpio.pi()
        o.pi.set_PWM_frequency(o.pateX, 50)
        o.pi.set_PWM_frequency(o.pateY, 50)
        o.pi.set_PWM_dutycycle(o.pateX, o.centreX)
        o.pi.set_PWM_dutycycle(o.pateY, o.centreY)
        o.fx = o.centreX
        o.fy = o.centreY
        o.actif= True

    def bye(o):
        pass

    def pause(o):
        # figer les servos
        o.actif= False

    def reprendre(o):
        if not(o.actif):
            o.pi.set_PWM_dutycycle(o.pateX, o.fx)
            o.pi.set_PWM_dutycycle(o.pateY, o.fy)
            o.actif= True
            
    def centre(o):
        o.fx = o.centreX
        o.fy = o.centreY
        o.position()
        
    def position(o):
        if o.fx > o.maxX:
            o.fx = o.maxX
        if o.fy > o.maxY:
            o.fy = o.maxY
        if o.fx < o.minX:
            o.fx = o.minX
        if o.fy < o.minY:
            o.fy = o.minY
        o.pi.set_PWM_dutycycle(o.pateX, o.fx)
        o.pi.set_PWM_dutycycle(o.pateY, o.fy)
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
        o.fy += o.quart * (-dy)
        o.position()
    def tourne(o, dx):
        o.fx += o.quart * dx
        o.position()
        

class oeuil:
    debug = True
    coef = 0.5 # reduction de l'image avant face_location. 1 -> 3.5s/ima, 0.5 -> 1s/ima
    def __init__(o, perif = 0, debug = True):
        o.camera = cv2.VideoCapture(perif)
        o.debug = debug

    def camera(o):
        return o.camera

    def boxvisage(o):
        (grabbed, frame) = o.read_last()
        if o.debug:
            print ('capture')
        boxes, frame = o.face_location(frame)
        return boxes, frame

    def aff(o, frame, wait = True, titre = 'oeuil'):
        cv2.imshow(titre, frame)
        if wait:
            cv2.waitKey()

    def cible_grand(o, boxes_frame):
        (boxes, frame) = boxes_frame
        if len(boxes) == 0:
            return (0,0), frame
        grand = max([x2-x1+y2-y1 for (y1,x2,y2,x1) in boxes])
        boxes_par_taille = { x2-x1+y2-y1 : (y1,x2,y2,x1) for (y1,x2,y2,x1) in boxes }
        (top, right, bottom, left) = boxes_par_taille[grand]
        cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)
        (h, l, _) = frame.shape
        # vecteur partant du centre de l'ecran dans la direction du centre du rectangle ciblé
        vx = -((left + right - l)/2/l)
        vy = ((top + bottom - h)/2/h)
        if o.debug:
            print (boxes)
            print(top, right, bottom, left)
            print ('vx {} / vy {}'.format(vx,vy))
        return (vx, vy),frame
        
    def cible_centre(o, boxes_frame):
        (boxes, frame) = boxes_frame

    def face_location(o, frame, trace = True):
        rgb = cv2.cvtColor(cv2.resize(frame, None, fx = o.coef, fy = o.coef), cv2.COLOR_BGR2RGB) # , interpolation = cv2.INTER_LINEAR INTER_AREA
        petites_boxes = face_recognition.face_locations(rgb, model='hog')
        boxes = []
        for coin in petites_boxes:
            boxe = tuple((np.array(coin)/o.coef).astype(int))
            boxes.append(boxe)
            (top, right, bottom, left) = boxe
            if trace:
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        return boxes, frame

    def visu_1ima(o, titre = 'oeuil', visage = False):
        (grabbed, frame) = o.read_last()
        if visage:
            boxes, frame = o.face_location(frame)
            if cv2.waitKey(1) & 0xFF == ord('x'):
                o.cible_grand((boxes,frame))
        cv2.imshow(titre, frame)

    def visu(o, titre = 'oeuil', visage = False):
        while cv2.waitKey(1) & 0xFF == ord('q'):
            pass
        for i in range(5):
            (grabbed, frame) = o.camera.read()
            cv2.imshow(titre, frame)
            cv2.waitKey(10)
        while cv2.waitKey(1) & 0xFF != ord('q'):
            o.visu_1ima(titre = titre, visage = visage)

    # vide le buffer avant de capturer une image
    def read_last(o, timeout = 30):
        # xx attendre une image net
        d=0
        f=0
        cpt=0
        t0 = time.time()
        while f-d < 0.01:
            pred = f-d
            d = time.time()
            if d - t0 > timeout:
                return False, np.array(np.zeros((2,2), dtype = np.uint8))
            o.camera.grab()
            f = time.time()
            cpt +=1
        #if o.debug:
        #    print('skiped ', cpt, 'delai ', f-d, 'delai pred ', pred)
        return o.camera.retrieve()
    
    def ferme(o, titre = 'oeuil'):
        cv2.destroyWindows(titre)
        
    def bye(o):
        o.camera.release()
        cv2.destroyAllWindows()
        
    def __del__(o):
        o.bye()
        

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

def poursuite(c, o):
    (vx, vy),frame = o.cible_grand(o.boxvisage())
    c.monte(-vy*0.4)
    c.tourne(vx*0.8)
    return frame

def test2():
    c=cou()
    o=oeuil()
    try:
        while True:
            frame = poursuite(c,o)
            #cv2.imshow('oeuil', frame)
            cv2.waitKey(10)
            time.sleep(0.5)
    except KeyboardInterrupt:
        o.bye()
        
if __name__ == "__main__":
    test()
    test2()

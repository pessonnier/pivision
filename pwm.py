import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
pwmH = GPIO.PWM(13, 50)
pwmV = GPIO.PWM(12, 50)
pwmH.start(5)
pwmV.start(4)

for p in range(20,120):
  pwmH.ChangeDutyCycle(p/10)
  time.sleep(0.1)
for p in range(120,20,-1):
  pwmH.ChangeDutyCycle(p/10)
  time.sleep(0.10)
pwmH.ChangeDutyCycle(6)

for p in range(40,80):
  pwmV.ChangeDutyCycle(p/10)
  time.sleep(0.10)
for p in range(80,40,-1):
  pwmV.ChangeDutyCycle(p/10)
  time.sleep(0.10)


pwmH.stop()
pwmV.stop()
GPIO.cleanup()

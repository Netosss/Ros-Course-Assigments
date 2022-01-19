import RPi.GPIO as GPIO 
from time import sleep
GPIO.setwarnings(False)

class Motor():
    def __init__(self, Ena, In1, In2):
        self.Ena = Ena
        self.In1 = In1
        self.In2 = In2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Ena, GPIO.OUT)
        GPIO.setup(self.In1, GPIO.OUT)
        GPIO.setup(self.In2, GPIO.OUT)
        self.pwm = GPIO.PWM(Ena, 100)
        self.pwm.start(0)
        
    def __del__(self):
        self.pwm.ChangeDutyCycle(0)
    
    def moveMotor(self, x=55, t=0):
        GPIO.output(self.In1, GPIO.LOW)  
        GPIO.output(self.In2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(100)
        sleep(0.3)
        print("moving engine")
        self.pwm.ChangeDutyCycle(x)
        #sleep(0.6)
        sleep(t)
    
    def stopMotor(self, t=0):
        self.pwm.ChangeDutyCycle(0)
        #sleep(0.6)
        #GPIO.cleanup()
        print("stopping engine")
        sleep(t)
        
    def PauseMotor(self, x=24):
        self.pwm.ChangeDutyCycle(0)
    
    def ContinueMotor(self, x=30):
        self.pwm.ChangeDutyCycle(100)
        sleep(0.3)
        self.pwm.ChangeDutyCycle(x)

class ServoMotor:
    def __init__(self, ServoIn):
        self.ServoIn = ServoIn
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ServoIn, GPIO.OUT)
        self.pwm = GPIO.PWM(self.ServoIn, 50)
        self.pwm.start(0)
    
    def moveServo(self, power=7.5):
        self.pwm.ChangeDutyCycle(power)
        
    def stopServo(self):
        self.pwm.ChangeDutyCycle(0)

    def SetAngle(self, angle):
        duty = angle / 18 + 2
        GPIO.output(self.ServoIn, True)
        self.pwm.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(self.ServoIn, False)
        self.pwm.ChangeDutyCycle(0)

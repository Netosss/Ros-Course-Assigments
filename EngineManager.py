from multiprocessing import Manager
import RPi.GPIO as GPIO 
from time import sleep
from motors import *
from gpiozero import Button
import atexit  
#defines for flipper usage:
d = 26
angle0 = 70
defaultAngles = [angle0 + d,  0]
angle1 = defaultAngles[0] + d + 1
defaultAngles[1] = angle1 + d -1
powersList = [32, 23]

# d = 26
# angle0 = 70
# angle1 = angle0 + d
# defaultAngles = [angle1 + d + 1,  0]
# defaultAngles[1] = defaultAngles[0] + d -1

# angle0 = 82
# angle1 = angle0 + d + 3
# defaultAngle = angle1 + d +2
# # defaultAngle = angle2 + d + 2

angles = [angle0, angle1]

#gpio defines:
ButtonIn = 21
ServoMotorIn = 4
servoIn = [11, 9]
servoPow = [8.5, 7.2]
Ena = 22
In1 = 27
In2 = 17
Enb = 26
In4 = 19
In3 = 13
Enc = 6
In5 = 5
In6 = 12

atexit.register(GPIO.cleanup)  

#Engine manager is manages two feeders and 1 servo(flipper).
###Today 30.11 the machines work with some help, what means you need to help feeder number 1 with roll it at start, and help servo
###	with tight his wires. 

def checkRoller():

	GPIO.setmode(GPIO.BCM)  
	GPIO.setup(11, GPIO.OUT, initial=False)  
	p = GPIO.PWM(11,50) #50HZ  
	p.start(0)  
	sleep(2)
	p.ChangeDutyCycle(7.4)   


class EnginesManager:
	def __init__(self):
		#self.servo = None
		self.motor1 = None
		self.motor2 = None
		self.motor3 = None
		self.servos = [None] * 2
	
	def runRoller(self, EngineNum, pow=7.4):
		self.servos[EngineNum] = ServoMotor(servoIn[EngineNum])
		self.servos[EngineNum].moveServo(power = pow)
	#engine num is the number of the machine , wooden is 1
	def startEngine(self, EngineNum, x=20, t=0):
		print('starting engine num: ', EngineNum)
		if EngineNum == 0:
			self.motor1 = Motor(Ena, In1, In2)
			self.motor1.moveMotor(t=t, x=x)
			self.runRoller(EngineNum, pow=servoPow[EngineNum])
		elif EngineNum == 1:
			self.motor2 = Motor(Enb, In4, In3)
			self.motor2.moveMotor(t=t, x=x)
			self.runRoller(EngineNum, pow=7.4)
		elif EngineNum == 2:
			self.motor3 = Motor(Enc, In5, In6)
			self.motor3.moveMotor(t=t, x=x)
   	#engine num is the number of the machine , wooden is 1
	def stopEngine(self, EngineNum):
		print('stopping engine num:', EngineNum)
		if EngineNum == 0:
			if self.motor1 is not None:
				self.motor1.stopMotor()
		elif EngineNum == 1:
			if self.motor2 is not None:
				self.motor2.stopMotor()
		elif EngineNum == 2:
			if self.motor3 is not None:
				self.motor3.stopMotor()
		self.stopRoller(EngineNum)

		# sleep(0.6)
		# self.Flip(defaultAngle)
   #1 is straight(wire direction) and -1 is right, initilized to staright

	def stopRoller(self, EngineNum):
		if EngineNum == 2 or self.servos[EngineNum] == None: return
		self.servos[EngineNum].stopServo()
	
	
	def pauseSlider(self, cs = 25, defaultAngle=defaultAngles[0]):
		if self.motor3 is not None:
			self.motor3.PauseMotor()
			self.Flip(defaultAngle)
			sleep(0.5)
			self.motor3.ContinueMotor(cs)
   
	def Flip(self, angle):
		servo = ServoMotor(ServoMotorIn)
		servo.SetAngle(angle)
		#servo.stopServo()
		#print('flipped! val: ', self._servoVal )
		#self.servo.detach()

	def StartSlider(self, n=20):
		self.startEngine(2, x=n)
  
	def StopSlider(self):
		self.stopEngine(2)


def StartMotor(gpio, t=1):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio, GPIO.OUT)
    GPIO.output(gpio, GPIO.HIGH)
    sleep(t)
    #GPIO.output(gpio, GPIO.LOW)
    #GPIO.cleanup()


# # # 5,32,55
def main():
# # 	# print('starting!!')
	# button = Button(ButtonIn)
	# button.wait_for_press()
# # 	#GPIO.cleanup()
	manager = EnginesManager()
# 	#manager.Flip(angle1)
# 	# manager.Flip(5)
# # 	# sleep(2)
# # 	# manager.Flip(32)
# # 	# sleep(2)
# # 	# manager.Flip(55)
# # 	# sleep(2)
	# manager.Flip(angle0)
	# manager.StartSlider(30)
	# # # manager.pauseSlider(2)
	manager.Flip(angle0)
	manager.Flip(defaultAngles[0])
	manager.Flip(angle1)
	manager.Flip(defaultAngles[1])
# 	manager.StopSlider()
# 	# manager.startEngine(2, x=20, t=2)
# 	# manager.stopEngine(3)s
# # 	#StartMotor(MotorIn, 5)
	manager.startEngine(0, x=32)
	manager.stopEngine(0)
# # 	# manager.Flip(33)
# # 	#sleep(5)
	manager.startEngine(1, x=23)
	manager.stopEngine(1)
	# manager.stopEngine(0)
	manager.StopSlider()
# # 	# manager.Flip(60)
# 	#sleep(2)
# 	#sleep(2)
	GPIO.setmode(GPIO.BCM)
	




if __name__ == '__main__':
    	main()
	

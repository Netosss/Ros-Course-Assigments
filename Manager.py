from EngineManager import *
from firebaseManager import *
from cameraManager import *
from VoiceManager import *
import threading
from time import sleep
from gpiozero import Button
import signal
from multiprocessing import Process


class Manager:
    def __init__(self):
        self.engine_manager = EnginesManager()
        self.fire_base_con = FireBaseManager()
        self.con_success = self.fire_base_con.Connect() #here we continue only if base got the amounts
        self.success=True
        self.voice=VoiceManager()
        # self.connected=self.run_with_limited_time(self.voice.setbt,('FC:58:FA:A5:C3:B8',),{},11)
        # if self.connected == True:
        #     self.voice.Say(",,,,,,,,,,,,,,,,,,,, heyy shalevvvvvvv")
        if self.con_success == False:
            #here we decide what to do in case of connection failure
            print("Connection failed or unsupport amounts was given")
            self.success=False
            return
        self.Camera = Camera()
        self.amounts = self.fire_base_con.amountList #return amounts in list accordingly
        self.engine_manager.StartSlider(n=40) #start vibration on slider
        sleep(2) #wait 2 seconds for stablization 
    def run_with_limited_time(self,func, args, kwargs, time):
        p = Process(target=func, args=args, kwargs=kwargs)
        p.start()
        p.join(time)
        if p.is_alive():
            p.terminate()
            return False
        return True

    def Type_Counting(self, number):
        print("start counting type_a, the given amount is: {}".format(self.amounts[number]))
        self.engine_manager.Flip(angles[number])
        self.engine_manager.startEngine(number, x=powersList[number])# start feeder
        self.Camera.Counting(self.amounts[number]) #wait to the camera finish counting
        self.engine_manager.stopEngine(number)#pause feeder
        sleep(2)# for letting the last screw fall into the flipper
        self.engine_manager.pauseSlider(defaultAngle=defaultAngles[number])# pause the slider and flip to default angle. 
        # sleep(3)
        

 
if __name__ == '__main__':
    # button = Button(ButtonIn)
    # button.wait_for_press()
    manager=Manager()
    if manager.success==True:
        manager.voice.Say("The amounts we have are {} from type 1 , and {} from type 2".format(manager.amounts[0],manager.amounts[1]))
        for i in range(len(manager.amounts)):
            manager.Type_Counting(i)
            sleep(5)
            manager.voice.Say("bla counting type {} done".format(i+1))
        manager.engine_manager.StopSlider() #start vibration on slider

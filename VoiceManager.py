import pyttsx3
import os
import time
import os,sys,time,pexpect
import subprocess
class VoiceManager:
    def __init__(self):
        # name = 'SB-01'
        # addr = 'FC:58:FA:A5:C3:B8'
        # port = 1
        # passkey = "1111"
        # subprocess.call("kill -9 'pidof bluetooth-agent'",shell=True)
        # status = subprocess.call("bluetooth-agent " + passkey + " &",shell=True)
        # try:
        #     s= bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        #     s.connect((adrr,port))
        # except bluetooth.btcommon.BluetoothError as err:
        #     print('failed')
        # subprocess.call("kill -9 'pidof bluetooth-agent'",shell=True)
        # cmd = 'bluetoothctl'
        # os.system(cmd)
        

        # subprocess.call("bluetoothctl && connect FC:58:FA:A5:C3:B8",shell=True)
        # time.sleep(0.2)
        # cmd2 = 'connect FC:58:FA:A5:C3:B8'
        # os.system(cmd2)
        # time.sleep(2)
        self.engine= pyttsx3.init()
        rate = self.engine.getProperty('rate')
        volume = self.engine.getProperty('volume')
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('rate',130)
        self.engine.setProperty('voice',voices[12].id)
        
    def Say(self,text):
        self.engine.say(text)
        self.engine.runAndWait()
        
    def setbt(self,address):
        response=''
        p = pexpect.spawn('bluetoothctl', encoding='utf-8')
        p.logfile_read = sys.stdout
        p.expect('#')
        p.sendline("remove "+address)
        p.expect("#")
        p.sendline("scan on")
        mylist = ["Discovery started","Failed to start discovery","Device "+address+" not available","Failed to connect","Connection successful"]
        while response != "Connection successful":
            p.expect(mylist)
            response=p.after
            p.sendline("connect "+address)
            time.sleep(1)
        p.sendline("quit")
        p.close()
        #time.sleep(1)
        return

if __name__ == '__main__':
    manager= VoiceManager()
    manager.setbt('FC:58:FA:A5:C3:B8')
    manager.Say("hey bob how do you fill today")
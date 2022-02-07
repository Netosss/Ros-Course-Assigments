import pyttsx3
import os
import time
import os,sys,time,pexpect
import subprocess
class VoiceManager:
    def __init__(self):
        self.engine= pyttsx3.init()
        rate = self.engine.getProperty('rate')
        volume = self.engine.getProperty('volume')
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('rate',110)
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
        return

# if __name__ == '__main__':
#     manager= VoiceManager()
#     manager.setbt('FC:58:FA:A5:C3:B8')
#     manager.Say("hey bob how do you fill today")
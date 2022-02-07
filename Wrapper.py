from gpiozero import Button
from EngineManager import ButtonIn
import os
if __name__ == '__main__':
    button = Button(ButtonIn)
    btc=1
    while True:
        button.wait_for_press()
        print("btc= {}".format(btc))
        cmd = "python3 Manager.py -btc {}".format(btc)
        os.system(cmd)
        btc=0



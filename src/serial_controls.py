import time
import serial

def foo():
    print("sent")
    ardu = serial.Serial('/dev/cu.usbmodem101',9600)
    time.sleep(1)
    ardu.write('s'.encode())
    time.sleep(1)
    #ardu.close()

while (true):
    foo()
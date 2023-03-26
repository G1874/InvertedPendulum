import serial
from serial.tools import list_ports
import collections
import numpy as np

class SerialCommunication():
    def __init__(self, Port:str='COM4',Baudrate=115200,CreateInstanceOfSerial:bool=True,
                 Parity='N',Bytesize=serial.EIGHTBITS,Stopbits=serial.STOPBITS_ONE,Timeout=0.1) -> None:
        """ serial class parameters """
        self.Port:str = Port
        self.Baudrate = Baudrate
        self.Parity = Parity
        self.Bytesize = Bytesize
        self.Stopbits = Stopbits
        self.Timeout = Timeout
        """ data receive loop condition """
        self.stp:bool = False
        """ data variables and lists """
        self.dist = 0
        self.ang = 0
        self.comNumber = 0
        self.errorNumber = 0
        self.distance = collections.deque(np.zeros(200))
        self.angle = collections.deque(np.zeros(200))

        if CreateInstanceOfSerial == True:
            try:
                self.createSerialInstance()
            except:
                print('error: cannot create new instance, check if parameters are correct')

    def createSerialInstance(self):
        self.serialInst = serial.Serial()
        self.serialInst.port = self.Port
        self.serialInst.baudrate = self.Baudrate
        self.serialInst.parity = self.Parity
        self.serialInst.bytesize = self.Bytesize
        self.serialInst.stopbits = self.Stopbits
        self.serialInst.timeout = self.Timeout
        
    def sendInstruction(self,instructionNrr:int,additionalArgument:int=255):
        if instructionNrr >= 0 and instructionNrr <= 255:
            if self.serialInst is not None:
                if not self.serialInst.is_open:
                    self.serialInst.open()
                communicate = [instructionNrr,additionalArgument]
                self.serialInst.write(communicate)
            else:
                print("error: serial instance does not exist")
        else:
            print("error: invalid instruction number")
    
    def startReceiving(self,saveToFile:bool=True):
        if self.serialInst is not None:
            if not self.serialInst.is_open:
                self.serialInst.open()
                self.stp = False
                self.distance = collections.deque(np.zeros(200))
                self.angle = collections.deque(np.zeros(200))
                self.comNumber = 3
                while not self.stp:
                    if self.serialInst.in_waiting > 0:
                        communicate = self.serialInst.read(1)
                        self.handleData(communicate)
        else:
            print('error: serial instance does not exist')

    def stopReceiving(self):
        self.stp = True
        if self.serialInst is not None:
            if self.serialInst.is_open:
                self.serialInst.close()
                self.comNumber = 2

    def handleData(self, communicate):
        marker = communicate.hex()
        """ value in hex """
        if marker == '61':
            data = self.serialInst.read(2).hex()
            self.dist = int(data[0:4],16)
            #self.ang = int(data[4:8],16)
            self.distance.popleft()
            self.angle.popleft()
            self.distance.append(self.dist)
            self.angle.append(0)
        """ value in hex """
        if marker == '65':
            data = self.serialInst.read(1).hex()
            self.errorNumber = (int(data,16))
        """ value in hex """
        if marker == '63':
            data = self.serialInst.read(1).hex()
            self.comNumber = (int(data,16))

    def getPortState(self):
        ports = list_ports.comports()
        for i in ports:
            if self.Port in i:
                return True
        return False

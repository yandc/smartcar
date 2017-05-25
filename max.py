import RPi.GPIO as GPIO
from constant import maxMod
import time

GPIO.setmode(GPIO.BOARD)

class Max7219:
    def __init__(self, pieceCount = 1):
        self.PinDin = 8
        self.PinCs = 10
        self.PinClk = 12
        self.pieceCount = pieceCount
        GPIO.setup(self.PinDin, GPIO.OUT)
        GPIO.setup(self.PinCs, GPIO.OUT)
        GPIO.setup(self.PinClk, GPIO.OUT)
        for i in range(pieceCount):
            self.write(9, 0, i)
            self.write(10, 3, i)
            self.write(11, 7, i)
            self.write(12, 1, i)
            self.write(15, 0, i)

    def delay(self):
        for i in xrange(10000):
            pass

    def writeByte(self, byte):
        r = 0x80
        GPIO.output(self.PinCs, False)
        for i in range(8):
            bit = byte & r
            GPIO.output(self.PinClk, False)
            GPIO.output(self.PinDin, bool(bit))
            GPIO.output(self.PinClk, True)
            r = r >> 1

    def write(self, addr, data, piece = 0):
        GPIO.output(self.PinCs, False)
        
        for i in range(self.pieceCount-piece-1):
            self.writeByte(0)
            self.writeByte(0)
            
        self.writeByte(addr)
        self.writeByte(data)
        
        for i in range(piece):
            self.writeByte(0)
            self.writeByte(0)
            
        GPIO.output(self.PinCs, True)

        
    def show(self, char, piece = 0):
        offset = ord(str.upper(char)) - ord('0')
        if offset > 9:
            offset = ord(str.upper(char)) - ord('A') + 10
        for i in range(8):
            self.write(i+1, maxMod[offset][i], piece)
            self.delay()

            
if __name__ == '__main__':
    max = Max7219(2)
    max.delay()
    for i in range(10):
        max.show(str(i), 1)
        for j in range(10):
            max.show(str(j))
            time.sleep(1)

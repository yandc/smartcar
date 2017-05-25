import os
import time
import RPi.GPIO as gpio
import struct
gpio.setmode(gpio.BOARD)
import threading

from cameraControler import cameraRun
class MotorCtrl:
    enablePin = 33
    aPin = 35
    bPin = 37
    minPwm = 20
    maxPwm = 100
    precise = 20
    aPinValue = False
    bPinValue = True
    brake = False
    reverseFlag = False
    def __init__(self):
        gpio.setup(self.enablePin, gpio.OUT)
        gpio.setup(self.aPin, gpio.OUT)
        gpio.setup(self.bPin, gpio.OUT)
        gpio.output(self.aPin, self.aPinValue)
        gpio.output(self.bPin, self.bPinValue)
        self.pwm = gpio.PWM(self.enablePin, 50)
        self.pwm.start(0)
        
    def ctrl(self, speed, brake=False):
        if brake == True:
            self.brake = True
            gpio.output(self.aPin, False)
            gpio.output(self.bPin, False)
            gpio.output(self.enablePin, True)
            return
        elif self.brake == True:
            self.brake = False
            gpio.output(self.aPin, self.aPinValue)
            gpio.output(self.bPin, self.bPinValue)
            gpio.output(self.enablePin, 0)
            return
            
        if abs(speed) > self.precise:
            if speed > 0:
                speed = self.precise
            else:
                speed = -self.precise
        if speed < 0 and self.reverseFlag == False:#change direction
            self.pwm.ChangeDutyCycle(0)
            gpio.output(self.aPin, self.bPinValue)
            gpio.output(self.bPin, self.aPinValue)
            self.reverseFlag = True
        elif speed > 0 and self.reverseFlag == True:
            self.pwm.ChangeDutyCycle(0)
            gpio.output(self.aPin, self.aPinValue)
            gpio.output(self.bPin, self.bPinValue)
            self.reverseFlag = False
            
        if speed == 0:
            self.pwm.ChangeDutyCycle(0)
        else:
            cycle = self.minPwm + max(abs(speed),1)*(self.maxPwm-self.minPwm)/self.precise
            self.pwm.ChangeDutyCycle(cycle)
        if abs(speed) < 1:
            time.sleep(speed)
            self.pwm.ChangeDutyCycle(0)

class ServoCtrl:
    servoPin = 12
    lastAngle = 0
    maxPwm = 11.7
    midPwm = 9.7
    minPwm = 7.1
    precise = 30
    leftPrecise = (midPwm-minPwm)/precise
    rightPrecise = (maxPwm - midPwm)/precise
    servoSpeed = 0.6 #time for 90 degree

    def __init__(self):
        gpio.setup(self.servoPin, gpio.OUT, initial=False)
        self.pwm = gpio.PWM(self.servoPin, 50) #50HZ
        self.pwm.start(0)
        self.ctrl(0, 0.5)

    def ctrl(self, angle, stime=0):
        if self.lastAngle == angle and stime == 0:
            return
        if abs(angle) > self.precise:
            if angle > 0:
                angle = self.precise
            else:
                angle = -self.precise
        if angle > 0:
            self.pwm.ChangeDutyCycle(self.midPwm + angle*self.rightPrecise)
        else:
            self.pwm.ChangeDutyCycle(self.midPwm + angle*self.leftPrecise)
        time.sleep(stime + self.servoSpeed*abs(self.lastAngle-angle)/self.precise)
        self.pwm.ChangeDutyCycle(0)
        self.lastAngle = angle

class Controler:
    evFmt = 'IhBB'
    evSize = struct.calcsize(evFmt)
    servoAngle = 0
    servoDirection = 0
    motorSpeed = 0
    motorForward = 1
    motorBrake = False
    maxAxisValue = 32767
    running = False
    cameraRunning = False
    status = 0
    servoThread = None
    motorThread = None
    cameraThread = None

    def __init__(self):
        self.fp = open('/dev/input/js0', 'rb')
        self.servoCtrl = ServoCtrl()
        self.motorCtrl = MotorCtrl()

    def ctrlServo(self):
        while True:
            if not self.running:
                self.servoCtrl.ctrl(0)
                break
            elif self.cameraRunning:
                time.sleep(0.5)
                continue
            self.servoCtrl.ctrl(self.servoAngle)
            time.sleep(0.1)

    def ctrlMotor(self):
        while True:
            if not self.running:
                self.motorCtrl.ctrl(0)
                break
            elif self.cameraRunning:
                time.sleep(0.5)
                continue
            if self.motorBrake == True:
                self.motorCtrl.ctrl(0, True)
            else:
                self.motorCtrl.ctrl(self.motorSpeed)
            time.sleep(0.1)

    def cameraCtrl(self):
        for angle in cameraRun():
            if not self.running or not self.cameraRunning:
                self.motorCtrl.ctrl(0)
                self.servoCtrl.ctrl(0)
                break
            self.servoCtrl.ctrl(angle)
            #self.motorCtrl.ctrl(max((self.servoCtrl.precise-abs(angle)), 3))
            #self.motorCtrl.ctrl(10/(abs(angle)+10)
            self.motorCtrl.ctrl(0.3)

    def run(self):
        while True:
            data = self.fp.read(self.evSize)
            time, value, typ, number = struct.unpack(self.evFmt, data)
            if typ == 2 and number == 0:#servo control, direction
                if value < 0:
                    self.servoDirection = -1
                elif value > 0:
                    self.servoDirection = 1
                else:
                    self.servoDirection = 0
                    self.servoAngle = 0
            if typ == 2 and number == 1:#scope
                self.servoAngle = self.servoDirection*int(ServoCtrl.precise*float(self.maxAxisValue+value)/(2*self.maxAxisValue))
            elif typ == 2 and number == 4:#speed
                percent = float(self.maxAxisValue+value)/(2*self.maxAxisValue)
                self.motorSpeed = int(self.motorForward*MotorCtrl.precise*percent)
            elif typ == 1 and number == 6:#backword
                if value == 1:
                    self.motorForward = -1
                else:
                    self.motorForward = 1
                self.motorSpeed = self.motorForward*abs(self.motorSpeed)
            elif typ == 1 and number == 1:#brake
                if value == 1:
                    self.motorBrake = True
                else:
                    self.motorBrake = False
            elif typ == 1 and number == 11:
                self.status = value
            elif typ == 1 and number == 3 and value == 1 and self.status == 1:#start
                if self.running == False:
                    self.running = True
                    self.servoThread = threading.Thread(target=self.ctrlServo)
                    self.motorThread = threading.Thread(target=self.ctrlMotor)
                    self.servoThread.start()
                    self.motorThread.start()
            elif typ == 1 and number == 0 and value == 1 and self.status == 1:#camera start/stop
                if self.running == True and self.cameraRunning == False:
                    self.cameraRunning = True
                    self.cameraThread = threading.Thread(target=self.cameraCtrl)
                    self.cameraThread.start()
                elif self.cameraRunning == True:
                    self.cameraRunning = False
                    if self.cameraThread:
                        self.cameraThread.join()
                        self.cameraThread = None
            elif typ == 1 and number == 4 and value == 1 and self.status == 1:#stop
                self.running = False
                if self.servoThread:
                    self.servoThread.join()
                    self.servoThread = None
                if self.motorThread:
                    self.motorThread.join()
                    self.motorThread = None
                if self.cameraThread:
                    self.cameraThread.join()
                    self.cameraThread = None
            elif typ == 1 and number == 7 and value == 1 and self.status == 1:#shutdown
                print '****************shutdown!!!'
                os.system('sudo shutdown -h 0')
            print 'angle:%s, speed:%s, brake:%s, running:%s'%(self.servoAngle, self.motorSpeed, self.motorBrake, self.running)

            
if __name__ == '__main__':
    ctrl = Controler()
    ctrl.run()
    print 'abc'

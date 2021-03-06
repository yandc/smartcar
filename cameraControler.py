import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import pdb
rows = 160
cols = 224
capSize = (cols, rows)
cap = None
def baseWeightFun(x):
    if abs(cols/2 - x) < cols/6:
        return 1
    else:
        return 1-abs(1-2.0*x/cols)
base = [baseWeightFun(x) for x in range(cols)]

def cameraClose():
    global cap
    if cap:
        cap.close()
        
def cameraRun():
    global cap
    if cap == None:
        cap = PiCamera()
        cap.resolution = capSize
        cap.brightness = 80
        cap.contrast = 60
    frame = PiRGBArray(cap, size=capSize)
    while True:
        now = time.time()
        #capture image
        frame.truncate(0)
        cap.capture(frame, format='bgr', use_video_port=True)
        gray = cv2.cvtColor(frame.array, cv2.COLOR_BGR2GRAY)
        #threashold and split track
        binAry1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 51, 7)
        kernel = np.ones((8,8), np.uint8)
        binAry2 = cv2.morphologyEx(binAry1,cv2.MORPH_OPEN,kernel)
        binAry3 = np.zeros(gray.shape, dtype=np.uint8)
        ctns, hi = cv2.findContours(binAry2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        maxArea = 0
        maxCtn = None
        for ctn in ctns:
            area = cv2.contourArea(ctn)
            if area > maxArea:
                maxArea = area
                maxCtn = ctn
        cv2.drawContours(binAry3, [maxCtn], -1, (255,255,255), -1)

        #split into 10 line
        track = []
        for i in xrange(len(binAry3), 0, -10):
            ysum = np.sum(binAry3[i-10:i], axis=0, dtype=np.int32)
            sect = [j for j, s in enumerate(ysum) if s > 1275]
            if len(sect) > 0:
                middle = np.sum(sect)/len(sect)
                track.append((i, middle))
        if len(track) == 0:
            xdiff = -cols/2
            ydiff = rows
        else:
            xdiff = float(track[-1][1] - cols/2)
            ydiff = float(track[-1][0])
        if abs(xdiff) < cols/12:
            yield 0
        else:
            yield xdiff*90*(320+ydiff)/(cols*480)


if __name__ == '__main__':
    for angle in cameraRun():
        time.sleep(0.1)

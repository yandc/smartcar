import cv2
import numpy as np
import time
img = cv2.imread('image.jpg', cv2.IMREAD_GRAYSCALE)
img1 = cv2.resize(img, (320, 240), interpolation=cv2.INTER_LINEAR)
now = time.time()
rows,cols=img1.shape
base = [(1-abs(1-2.0*x/cols)) for x in range(cols)]
M1=cv2.getRotationMatrix2D((cols/2,rows),30, 0.6)
M2=cv2.getRotationMatrix2D((cols/2,rows),0, 0.6)
M3=cv2.getRotationMatrix2D((cols/2,rows),-30, 0.6)
img2=cv2.warpAffine(img1,M1, (cols, rows))
img3=cv2.warpAffine(img1,M2, (cols, rows))
img4=cv2.warpAffine(img1,M3, (cols, rows))
imgs = [img1, img2, img3, img4]
for i in range(len(imgs)):
    ysum = np.sum(imgs[i], axis=0, dtype=np.int32) * base
    value = np.sum(ysum, axis=0, dtype=np.int32)
    print value
print time.time() - now


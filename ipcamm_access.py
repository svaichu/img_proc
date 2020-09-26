

import cv2
import numpy as np


cap = cv2.VideoCapture('http://192.168.0.102:8080/video')
while(True):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


from __future__ import print_function

import cv2 as cv
import numpy as np
import time


max_lowThreshold = 100
window_name = 'Edge Map'
title_trackbar = 'Min Threshold:'
ratio = 3
kernel_size = 2
def CannyThreshold(val):
    low_threshold = val
    detected_edges = cv.Canny(src_gray, low_threshold, low_threshold*ratio, kernel_size)
    mask = detected_edges != 0
    dst = src * (mask[:,:,None].astype(src.dtype))
    cv.imshow(window_name, dst)

cv.namedWindow(window_name)
cv.createTrackbar(title_trackbar, window_name , 0, max_lowThreshold, CannyThreshold)

# src = cv.imread('test_4.jpg')
# if src is None:
#     print('Could not open or find the image: ')
#     exit(0)

cap = cv.VideoCapture('https://192.168.0.102:8080/video')
while(True):
    ret, src = cap.read()
    src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    CannyThreshold(100)
    # cv.imshow('frame',frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
         break
    #time.sleep(1)

cap.release()
cv.destroyAllWindows()

#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np

src = cv.imread("test_4.jpg")
if src is None:
    print('Could not open or find the image:', args.input)
    exit(0)
gray = cv.cvtColor(src,cv.COLOR_BGR2GRAY)
edges = cv.Canny(gray,50,150,apertureSize = 3)
# stretch = cv.resize(src, (780, 540), interpolation = cv.INTER_NEAREST)

cv.imshow('Source image', edges)
cv.waitKey()

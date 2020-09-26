from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np

src = cv.imread("lena.jpg")
if src is None:
    print('Could not open or find the image:', args.input)
    exit(0)

cv.imshow('Source image', src)
cv.waitKey()

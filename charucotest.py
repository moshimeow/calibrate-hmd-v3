import time
import cv2
import cv2.aruco
import numpy as np
import random
import multiprocessing
import traceback
import sys
import os
import shutil
import math

import argparse

squaresize = 0.04105
arucosize = squaresize*.8
checkerboardWidth = 12
checkerboardHeight = 8

monitorWidthPx = 1280
monitorHeightPx = 720
bine = 15

numImagesRequired = (checkerboardWidth-1)*(checkerboardHeight-1)


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
board = cv2.aruco.CharucoBoard_create(checkerboardWidth,checkerboardHeight,squaresize,arucosize,dictionary)
img = board.draw((monitorWidthPx,monitorHeightPx-(2*bine)))

img = np.vstack((np.ones((bine,monitorWidthPx)), img, np.ones((bine,monitorWidthPx))))


# cv2.namedWindow("oh yeah", 0)
# cv2.moveWindow("oh yeah", 2200, 100)
# cv2.setWindowProperty("oh yeah", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

cv2.imshow('oh yeah',img)
cv2.waitKey(0)
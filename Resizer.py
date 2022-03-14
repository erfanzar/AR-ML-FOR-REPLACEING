import numpy as np
import cv2 as cv


def Resize(Pic,size):
    picd = cv.resize(Pic,(size,size),None)
    return picd
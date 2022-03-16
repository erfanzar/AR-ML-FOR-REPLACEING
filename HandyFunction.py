import cv2 as cv

import numpy as np

def LmFinder(camera, result):

    lmlist = []

    if result.multi_hand_landmarks:

        myhand = result.multi_hand_landmarks[0]

        for id, lm in enumerate(myhand.landmark):

            h, w, _ = camera.shape

            x = int(lm.x * w)

            y = int(lm.y * h)

            lmlist.append([id, x, y])

    return lmlist




def overlayPNG(imgBack, imgFront, pos=[0, 0]):

    hf, wf, cf = imgFront.shape

    hb, wb, cb = imgBack.shape

    *_, mask = cv.split(imgFront)

    Frong = cv.cvtColor(imgFront , cv.COLOR_RGB2BGRA)

    maskBGRA = cv.cvtColor(mask, cv.COLOR_GRAY2BGRA)

    maskBGR = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)

    imgRGBA = cv.bitwise_and(Frong, maskBGRA)

    imgRGB = cv.cvtColor(imgRGBA, cv.COLOR_BGRA2BGR)



    imgMaskFull = np.zeros((hb, wb, cb), np.uint8)

    imgMaskFull[pos[1]:hf + pos[1], pos[0]:wf + pos[0], :] = imgRGB

    imgMaskFull2 = np.ones((hb, wb, cb), np.uint8) * 255

    maskBGRInv = cv.bitwise_not(maskBGR)

    imgMaskFull2[pos[1]:hf + pos[1], pos[0]:wf + pos[0], :] = maskBGRInv



    imgBack = cv.bitwise_and(imgBack, imgMaskFull2)

    imgBack = cv.bitwise_or(imgBack, imgMaskFull)



    return imgBack



def rotateImage(img, angle, scale=1):

    h, w = img.shape[:2]

    center = (w / 2, h / 2)

    rotate_matrix = cv.getRotationMatrix2D(center=center, angle=angle, scale=scale)


    img = cv.warpAffine(src=img, M=rotate_matrix, dsize=(w, h))


    return img
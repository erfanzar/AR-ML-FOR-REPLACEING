import cv2 as cv

import numpy as np

import mediapipe as mp

import time

from HandyFunction import LmFinder


from HandyFunction import overlayPNG

from HandyFunction import rotateImage


from Resizer import Resize

cam = cv.VideoCapture(0)

frontRing = cv.imread("FRONTONE.png")


BackSideRing = cv.imread("G.jpg" , cv.IMREAD_UNCHANGED)

hf, wf, cf = BackSideRing.shape


frontRing = Resize(frontRing,100)

BackSideRing = Resize(BackSideRing,100)


handp = mp.solutions.hands

Hands = handp.Hands()

Connections = handp.HAND_CONNECTIONS


class drawer:
    def __init__(self ,lst):
        self.fx = lst[0]
        self.fy = lst[1]

    def drawer(self, frame):
        cv.circle(frame ,(self.fx,self.fy),10,(250,150,0),6)
ptime = 0


while True:

    _, frame = cam.read()

    feedimage = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

    result = Hands.process(feedimage)

    ntime = time.time()

    fps = 1 / (ptime - ntime)

    ptime = ntime

    fps = int(fps)

    lmlist = LmFinder(feedimage, result)

    if len(lmlist) != 0:

        x0 = lmlist[14][1]

        y0 = lmlist[14][2]

        x1 = lmlist[13][1]

        y1 = lmlist[13][2]

        fx4 = lmlist[4][1]

        fy4 = lmlist[4][2]

        fx8 = lmlist[8][1]

        fy8 = lmlist[8][2]

        fx12 = lmlist[12][1]

        fy12 = lmlist[12][2]

        fx16 = lmlist[16][1]

        fy16 = lmlist[16][2]

        fx20 = lmlist[20][1]

        fy20 = lmlist[20][2]

        fingers = [[fx4 , fy4],[fx8 , fy8],[fx12 , fy12],[fx16 , fy16],[fx20 , fy20]]

        # paranoid1 = int(np.interp(y0 + 30, [y0, y0 + 50], [5, 15]))

        readdx = x0 - x1

        readdy = y0 - y1

        drw = [drawer(i) for i in fingers]

        drawnigga = [drawer.drawer(i,frame) for i in drw]


        hs = x0-50

        ws = y0-50


        if 150<fy8 & fy12 & fy16<450:

            cv.circle(frame, (x0 - readdx, y0 - readdy), 10, (255, 255, 0),1)

            # if x0 & y0 != 0:
            #     print("state1")
            #     print(x0 , y0)
            if 50<ws<430 and 50<hs<590:

                ffr = overlayPNG(frame[ws:ws+100 ,hs:hs+100,:] ,frontRing)
                

                frame[ws:ws+100 ,hs:hs+100,:] = ffr

                # [ws:ws+100 ,hs:hs+100,:]

                print("IN TarGEt !")

                # print(f"frame : {type(frame[ws:ws+100 ,hs:hs+100,:])} , {frame[ws:ws+100 ,hs:hs+100].shape}")


                # print(f"FrontRing : {type(frontRing)} , {frontRing.shape}")

                # except:

                #     raise Exception("BRakeAt Point 2")
        if fx20>fx12>fx4 and fy4>fy8 :

            cv.ellipse(
                frame,
                (x0 , y0+20),
                (30,10),
                0,
                0,
                200,
                (255,200,0),
                1)

            # print("Left")

            """
                Detection on The LEft Nigga Hands
            """

        if fx20<fx12<fx4 and fy4>fy8 :

            cv.ellipse(
                frame,
                (x0 , y0+20),
                (30,10),
                0,
                0,
                200,
                (255,200,0),
                1)

            # print("Right")

            """
            Detection on The Right Nigga Hands
            """


        # print(f"lmList : {lmlist}")

        # print(f"X,Y : {x0,y0}")

    cv.putText(frame, f"Fps : {fps}", (60, 60), cv.FONT_ITALIC, 1, (200, 0, 0))

    cv.imshow("Test", frame)

    cv.waitKey(1)

    if cv.waitKey(1) == ord("q"):

        break

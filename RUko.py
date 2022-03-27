### Start of importing dependecies

import cv2 as cv

import numpy as np

import mediapipe as mp

import time

from HandyFunction import LmFinder


from HandyFunction import overlayPNG
from HandyFunction import rotateImage

from HandyFunction import calculate_rotation

from Resizer import Resize


### ending of importing dependecis

###  change able for the each device depend on which camera u want to use

class Ai:

    def __init__(self,front = "Images\FRONTONE.png" ,top = "Images\TOPSIDE.png"  , cameraindex = 0):
        self.front = front
        self.top = top
        self.cameraindex = cameraindex

    def forward(self):

        cam = cv.VideoCapture(self.cameraindex)



        ####  load the images from the images directory in the path


        TOPSIDE = cv.imread( self.front, cv.IMREAD_UNCHANGED)


        frontRing = cv.imread( self.top, cv.IMREAD_UNCHANGED)

        hf, wf, cf = TOPSIDE.shape


        # frontRing = Resize(frontRing,100)

        # TOPSIDE = Resize(TOPSIDE,100)


        handp = mp.solutions.hands

        Hands = handp.Hands()

        Connections = handp.HAND_CONNECTIONS


        ### drawing utils

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

            fps = 1 / (ntime - ptime)

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
                    if 60<ws<430 and 60<hs<590:

                        pfp = (fx16+fx12)

                        print(pfp)

                        intpertor = np.interp(pfp,[1000,200],[100,70])

                        intpertor = int(intpertor)

                        frontRing = Resize(frontRing,intpertor)


                        ffr = overlayPNG(frame[y0-20:y0-20+intpertor ,x0-20:x0-20+intpertor,:] ,frontRing)


                        frame[y0-20:y0-20+intpertor ,x0-20:x0-20+intpertor,:] = ffr

                        # [ws:ws+100 ,hs:hs+100,:]

                        print("IN TarGEt !")

                        # print(f"frame : {type(frame[ws:ws+100 ,hs:hs+100,:])} , {frame[ws:ws+100 ,hs:hs+100].shape}")


                        # print(f"FrontRing : {type(frontRing)} , {frontRing.shape}")

                        # except:

                        #     raise Exception("BRakeAt Point 2")
                elif fx20>fx12>fx4 and fy4>fy8 :

                    cv.ellipse(
                        frame,
                        (x0 , y0+20),
                        (30,10),
                        0,
                        0,
                        200,
                        (255,200,0),
                        1)

                    print("Left")





                    if 60<ws<430 and 60<hs<590:

                        pfp = (fx16+fx12)

                        print(pfp)

                        intpertor = np.interp(pfp,[1300,200],[100,70])

                        intpertor = int(intpertor)

                        TOPSIDE = Resize(TOPSIDE,intpertor)


                        ffr = overlayPNG(frame[y0+30:y0+30+intpertor ,x0-50:x0-50+intpertor,:] ,TOPSIDE)


                        frame[y0+30:y0+30+intpertor ,x0-50:x0-50+intpertor,:] = ffr

                        # [ws:ws+100 ,hs:hs+100,:]

                        print("IN TarGEt !")


                        # rotation = calculate_rotation(x0-50,fx16)

                        # print(rotation)


                        # print(f"frame : {type(frame[ws:ws+100 ,hs:hs+100,:])} , {frame[ws:ws+100 ,hs:hs+100].shape}")


                        # print(f"TOPSIDE : {type(TOPSIDE)} , {TOPSIDE.shape}")

                        # except:

                        #     raise Exception("BRakeAt Point 2")

                    print("LEft")



                    """
                        Detection on The LEft Nigga Hands
                    """

                elif fx20<fx12<fx4 and fy4>fy8 :

                    cv.ellipse(
                        frame,
                        (x0 , y0+20),
                        (30,10),
                        0,
                        0,
                        200,
                        (255,200,0),
                        1)

                    if 60<ws<430 and 60<hs<590:

                        pfp = (fx16+fx12)

                        print(pfp)

                        intpertor = np.interp(pfp,[1300,200],[100,70])

                        intpertor = int(intpertor)

                        TOPSIDE = Resize(TOPSIDE,intpertor)


                        ffr = overlayPNG(frame[y0+30:y0+30+intpertor ,x0-50:x0-50+intpertor,:] ,TOPSIDE)


                        frame[y0+30:y0+30+intpertor ,x0-50:x0-50+intpertor,:] = ffr

                        # [ws:ws+100 ,hs:hs+100,:]

                        print("IN TarGEt !")

                        # print(f"frame : {type(frame[ws:ws+100 ,hs:hs+100,:])} , {frame[ws:ws+100 ,hs:hs+100].shape}")


                        # print(f"TOPSIDE : {type(TOPSIDE)} , {TOPSIDE.shape}")

                        # except:

                        #     raise Exception("BRakeAt Point 2")

                        # rotation = calculate_rotation(x0-50,fx16)

                        # print(rotation)

                    print("Right")

                    """
                    Detection on The Right Nigga Hands
                    """


                # print(f"lmList : {lmlist}")

                # print(f"X,Y : {x0,y0}")

            cv.putText(frame, f"Fps : {fps}", (60, 60), cv.FONT_ITALIC, 1, (200, 0, 0))

            # cv.imshow("Test", frame)


            cv.waitKey(1)

            if cv.waitKey(1) == ord("q"):

                break
            return frame


        # MIT License

        # Copyright (c) 2022 Erano-

        # Permission is hereby granted, free of charge, to any person obtaining a copy
        # of this software and associated documentation files (the "Software"), to deal
        # in the Software without restriction, including without limitation the rights
        # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
        # copies of the Software, and to permit persons to whom the Software is
        # furnished to do so, subject to the following conditions:

        # The above copyright notice and this permission notice shall be included in all
        # copies or substantial portions of the Software.

        # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
        # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
        # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
        # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
        # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
        # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
        # SOFTWARE.


if __name__ == "__main__":
    Ai().forward()
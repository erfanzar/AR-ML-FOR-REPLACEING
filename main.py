### Start of importing dependecies

import cv2 as cv

import numpy as np

import mediapipe as mp

import time

from kivymd.app import MDApp

from kivymd.uix.button import MDRaisedButton

from kivy.uix.image import Image

from kivymd.uix.label import MDLabel

from kivy.clock import Clock

from kivymd.uix.boxlayout import MDBoxLayout

from kivy.graphics.texture import Texture


### ending of importing dependecis

### Functions

def Resize(Pic,size):
    picd = cv.resize(Pic,(size,size),None)
    return picd


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

    Frong = cv.cvtColor(imgFront, cv.COLOR_RGB2BGRA)

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


def calculate_rotation(a, b):
    """ return rotation angle from vector a to vector b, in degrees.
    Args:
        a : np.array vector. format (x,y)
        b : np.array vector. format (x,y)
    Returns:
        angle [float]: degrees. 0~360
    """
    unit_vector_1 = a / np.linalg.norm(a)
    unit_vector_2 = b / np.linalg.norm(b)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    angle = angle / np.pi * 180
    c = np.cross(b, a, 1, 1, 1, 1)
    if c > 0:
        angle += 180

    return angle


###  change able for the each device depend on which camera u want to use

class Ai:

    def __init__(self, capture, front="Images\FRONTONE.png", top="Images\TOPSIDE.png"):
        self.front = front
        self.top = top
        self.capture = capture

    def forward(self):
        TOPSIDE = cv.imread( self.front, cv.IMREAD_UNCHANGED)
        frontRing = cv.imread( self.top, cv.IMREAD_UNCHANGED)
        handp = mp.solutions.hands
        Hands = handp.Hands()
        class drawer:
            def __init__(self ,lst):
                self.fx = lst[0]
                self.fy = lst[1]
            def drawer(self, frame):
                cv.circle(frame ,(self.fx,self.fy),10,(250,150,0),6)
        ptime = 0
        while True:
            _, frame = self.capture.read()
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
                readdx = x0 - x1
                readdy = y0 - y1
                drw = [drawer(i) for i in fingers]
                drawnigga = [drawer.drawer(i,frame) for i in drw]
                hs = x0-50
                ws = y0-50
                if 150<fy8 & fy12 & fy16<450:
                    cv.circle(frame, (x0 - readdx, y0 - readdy), 10, (255, 255, 0),1)
                    if 60<ws<430 and 60<hs<590:
                        pfp = (fx16+fx12)
                        print(pfp)
                        intpertor = np.interp(pfp,[1000,200],[100,70])
                        intpertor = int(intpertor)
                        frontRing = Resize(frontRing,intpertor)
                        ffr = overlayPNG(frame[y0-20:y0-20+intpertor ,x0-20:x0-20+intpertor,:] ,frontRing)
                        frame[y0-20:y0-20+intpertor ,x0-20:x0-20+intpertor,:] = ffr
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
                    if 60<ws<430 and 60<hs<590:
                        pfp = (fx16+fx12)
                        print(pfp)
                        intpertor = np.interp(pfp,[1300,200],[100,70])
                        intpertor = int(intpertor)
                        TOPSIDE = Resize(TOPSIDE,intpertor)
                        ffr = overlayPNG(frame[y0+30:y0+30+intpertor ,x0-50:x0-50+intpertor,:] ,TOPSIDE)
                        frame[y0+30:y0+30+intpertor ,x0-50:x0-50+intpertor,:] = ffr
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
            cv.putText(frame, f"Fps : {fps}", (60, 60), cv.FONT_ITALIC, 1, (200, 0, 0))
            cv.waitKey(1)
            if cv.waitKey(1) == ord("q"):
                break
            return frame



class Application(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.frame = None
        self.web_cam = None
        self.Ai = None
        self.front = "Images/FRONTONE.png"
        self.top = "Images/TOPSIDE.png"

    def build(self):

        screen = MDBoxLayout(orientation='vertical')
        self.image = Image(size_hint=(1, 1))
        self.capture = cv.VideoCapture(0)
        self.Ai = Ai
        screen.add_widget(self.image)
        self.ptime = 0
        Clock.schedule_interval(self.update, 1.0/30.0)
        return screen

    def update(self, *args):
        TOPSIDE = cv.imread(self.front, cv.IMREAD_UNCHANGED)
        frontRing = cv.imread(self.top, cv.IMREAD_UNCHANGED)
        handp = mp.solutions.hands
        Hands = handp.Hands()

        _, frame = self.capture.read()
        feedimage = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        result = Hands.process(feedimage)
        ntime = time.time()
        fps = 1 / (ntime - self.ptime)
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
            fingers = [[fx4, fy4], [fx8, fy8], [fx12, fy12], [fx16, fy16], [fx20, fy20]]
            readdx = x0 - x1
            readdy = y0 - y1
            # drw = [drawer(i) for i in fingers]
            # drawnigga = [drawer.drawer(i, frame) for i in drw]
            hs = x0 - 50
            ws = y0 - 50
            if 150 < fy8 & fy12 & fy16 < 450:
                cv.circle(frame, (x0 - readdx, y0 - readdy), 10, (255, 255, 0), 1)
                if 60 < ws < 430 and 60 < hs < 590:
                    pfp = (fx16 + fx12)
                    print(pfp)
                    intpertor = np.interp(pfp, [1000, 200], [100, 70])
                    intpertor = int(intpertor)
                    frontRing = Resize(frontRing, intpertor)
                    ffr = overlayPNG(frame[y0 - 20:y0 - 20 + intpertor, x0 - 20:x0 - 20 + intpertor, :], frontRing)
                    frame[y0 - 20:y0 - 20 + intpertor, x0 - 20:x0 - 20 + intpertor, :] = ffr
            elif fx20 > fx12 > fx4 and fy4 > fy8:
                if 60 < ws < 430 and 60 < hs < 590:
                    pfp = (fx16 + fx12)
                    print(pfp)
                    intpertor = np.interp(pfp, [1300, 200], [100, 70])
                    intpertor = int(intpertor)
                    TOPSIDE = Resize(TOPSIDE, intpertor)
                    ffr = overlayPNG(frame[y0 + 30:y0 + 30 + intpertor, x0 - 50:x0 - 50 + intpertor, :], TOPSIDE)
                    frame[y0 + 30:y0 + 30 + intpertor, x0 - 50:x0 - 50 + intpertor, :] = ffr
            elif fx20 < fx12 < fx4 and fy4 > fy8:
                if 60 < ws < 430 and 60 < hs < 590:
                    pfp = (fx16 + fx12)
                    print(pfp)
                    intpertor = np.interp(pfp, [1300, 200], [100, 70])
                    intpertor = int(intpertor)
                    TOPSIDE = Resize(TOPSIDE, intpertor)
                    ffr = overlayPNG(frame[y0 + 30:y0 + 30 + intpertor, x0 - 50:x0 - 50 + intpertor, :], TOPSIDE)
                    frame[y0 + 30:y0 + 30 + intpertor, x0 - 50:x0 - 50 + intpertor, :] = ffr
        cv.putText(frame, f"Fps : {fps}", (60, 60), cv.FONT_ITALIC, 1, (200, 0, 0))
        self.frame = frame
        buffer = cv.flip(frame, 0).tobytes()
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
        texture.blit_buffer(buffer, colorfmt='bgr', bufferfmt='ubyte')
        self.image.texture = texture



if __name__ == "__main__":

    Application().run()
    
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
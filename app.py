import cv2 as cv

from RUko import Ai

from kivy.app import App

from kivy.uix.button import Button

from kivy.uix.image import Image

from kivy.uix.label import Label

from kivy.clock import Clock

from kivy.uix.boxlayout import BoxLayout

from kivy.graphics.texture import Texture


class Application(App):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)

        self.button = None

        self.web_cam = None

        self.varification = None

        self.ai = None

    def build(self):

        screen = BoxLayout(orientation='vertical')

        self.web_cam = Image(size_hint=(1,.9))

        self.button = Button(text="Start" , size_hint=(1,.2))

        self.varification = Label(text="NonDetected",size_hint=(1,.1))

        self.ai = Ai(cameraindex=0)

        screen.add_widget(self.web_cam)

        screen.add_widget(self.button)

        screen.add_widget(self.varification)

        Clock.schedule_interval(self.update,1.0/15.0)

        return screen

    def update(self,*args):

        frame = self.ai.forward()

        _height = frame.shape[0]

        _width = frame.shape[1]

        _chanels = frame.shape[2]

        buf = cv.flip(frame, 0).tobytes()

        img_texture = Texture.create(size=(_width,_height),colorfmt='bgr')

        img_texture.blit_buffer(buf,colorfmt='bgr',bufferfmt='ubyte')

        self.web_cam = img_texture



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
from kivy.app import App
from kivy.uix.camera import Camera
from kivy.uix import *
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget


class BoxLayOut(BoxLayout):
    pass
    # def __init__(self, **kwargs):
    #     super().__init__(**kwargs)
    #
    #     self.orientation = "vertical"
    #
    #     bu1 = Button(text="Hallo")
    #
    #     bu2 = Button(text="Hello")
    #
    #     self.add_widget(bu1)
    #
    #     self.add_widget(bu2)


class Camerad(Camera):
    pass


class MainWidget(Widget):
    pass


class AppScreen(App):
    pass


AppScreen().run()

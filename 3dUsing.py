from curses import window
from sklearn import exceptions
import slt

# from slt import mesh

import numpy as np

import glfw

# ym = mesh.Mesh.from_file('somefile.stl')

# data = np.zeros(100 , dtype=mesh.Mesh.dtype)

# ym = mesh.Mesh(data,remove_empty_areas=False)


if not glfw.init():
    raise Exception("GLFW Can not pass initation's")


windows = glfw.create_window(1270,780,'None',None)

if not window:
    glfw.terminate()
    raise Exception("Windows Terminated And Couldn't be Runn")

glfw.make_context_current(windows)

while not glfw.window_should_close():
    glfw.poll_events()
    glfw.swap_buffers()
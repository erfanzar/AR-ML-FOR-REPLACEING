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
from modulefinder import Module
import torch 
import torch.nn as nn
import torch.optim as optim
from torch import tensor
import torch.utils as utils
from torch.nn import Conv2d , ReLU , SiLU ,Linear
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

def autopad(k , p=None):
    if p != None:
        p = k // 2 if  isinstance(k , int) else [x // 2 for x in k]
        
    return k


class conv:
    def __init__(self ,c1,c2,k,s=1,p=None , act=True):
        self.conv = Conv2d(c1,c2,k,s,autopad(k,p))
        self.bt = nn.BatchNorm2d(c2)
        self.ac = SiLU if act is True else (act if isinstance(act , nn.Module) else nn.Identity())
        
        
        
        

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
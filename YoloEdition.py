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
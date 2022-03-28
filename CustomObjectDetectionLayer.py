from modulefinder import Module

import torch 

import torch.nn as nn

import torch.optim as optim

from torch import tensor

import torch.utils as utils

from torch.nn import Conv2d , ReLU , SiLU ,Linear

# import cv2 as cv


import numpy as np

from matplotlib import pyplot as plt

from config import config


class CNN(nn.Module):

    def __init__(self,c1,c2,act=True,**kwargs):
        
        super().__init__()

        self.conv = nn.Conv2d(c1,c2,bias= True if act == True else False , **kwargs)


        self.batch = nn.BatchNorm2d(c2)

        
        self.activation = nn.SiLU() if act == True else nn.Identity()


        self.act = act


    def forward(self , x):

        if self.act :
            
            return self.activation(self.batch(self.conv(x)))

        else :

            return self.conv(x)





class Residual(nn.Module):

    def __init__(self , c , use=True , num=1):

        super().__init__()

        self.layers = nn.ModuleList()

        for repeat in range(num) :

            self.layers += [
                nn.Sequential(
                    CNN(c , c * 2 , kernel_size=1),
                    CNN(c * 2 , c , kernel_size=3 , padding=1)
                )
            ] 


        self.num = num

        self.use = use


    def forward(self , x):

        for layer in self.layers :


            x = layer(x) + x if self.use == True else layer(x)


        return x

class ScalePrediction(nn.Module):
    
    def __init__(self ,c ,num):
       
        super().__init__()

        self.pred = nn.Sequential(
            CNN(c , c*2 , kernel_size=3 , padding=1),
            CNN(c*2 ,3 * (n+5), act=False , kernel_size=1)
        
        )  

        self.num = n

        self.c = c

    def forward(self , x):

        return (
            self.pred(x)
            .reshape(x.shape[0] , 3 , self.num + 5 , x.shape[2] , x.shape[3])
            .permute(0,1,3,4 , 2)
        )

class Detector(nn.Module):

    def __init__(self , numclass , in_c = 1):
        
        super().__init__()
        
        self.fullylayer = layer_creator()
        
        self.classes = numclass

        self.in_c = in_c







    def forward(self , x):

        out = []

        route_connections = []


        for layer in self.fullylayer:


            if isinstance(layer , ScalePrediction):

                out.append(layer(x))

                continue

            x = layer(x)

            if isinstance(layer , Residual) and layer.num_rep == 8 :

                route_connections.append(x)

            elif isinstance(layer , nn.Upsample):
                
                x = torch.cat([x,route_connections[-1]] , dim=1)

                route_connections.pop()







    def layer_creator(self):

        layers = nn.ModuleList()

        in_c = self.in_c
        
        for module in config:
        
            if isinstance(module , tuple):

                out_c , kernel_size , padding = module

                layers.append(
                    CNN(in_c , out_c , kernel_size=kernel_size , padding=padding)
                # )
                
                in_c = out_c

            elif isinstance(module , list):

                num_rep = module[1]

                layers.append(Residual(in_c,num=num_rep))
        
            elif isinstance(module , str):
                if module == "S":

                    layers +=[
                    
                        nn.Sequential(

                            Residual(in_c , use=False, num=1),
                        
                            CNN(in_c , in_c//2 ,kernel_size=1),
                        
                            ScalePrediction(in_c//2 , num=self.classes)

                        )
                    ]

                    in_c = in_c // 2


                elif module == "U":

                    layers.append(nn.Upsample(scale_factor=2))

                    in_c = in_c *3

        return layers








if __name__ == "__main__":
    model = Detector(2)
    print(model)


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
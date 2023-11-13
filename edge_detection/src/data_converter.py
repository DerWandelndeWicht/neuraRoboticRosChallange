#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import Image

class dataConverter:
    def __init__(self) -> None:
        pass

    def img2rosImage(array):

        img = Image
        lst = np.zeros((array.shape[0]*array.shape[1]*array.shape[2],))
        for x in range(array.shape[0]):
            lst.append(array[x,:,0],array[x,:,1],array[x,:,2])
        
        img.data = np.uint8(lst)
        img.width, img.height = array.shape[:2]
        img.step = array.shape[2] * array.shape[0]

        return img
    
    def rosImage2img(rosImg):

        img = np.zeros((rosImg.width, rosImg.height, 3))
        for cnt in range(len(rosImg / rosImg.height)):
            img[cnt,:,:] = rosImg.data[cnt*rosImg.height*3:(cnt+1)* rosImg.height*3]
        return img
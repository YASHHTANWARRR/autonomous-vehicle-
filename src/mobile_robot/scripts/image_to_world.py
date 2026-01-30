import cv2 
import numpy as np 
import yaml 
import os 

img_path='/home/hornet/autonomous-vehicle-/src/mobile_robot/maps/pasted_image.png'
out_path='/home/hornet/autonomous-vehicle-/src/mobile_robot/maps/'
map_name ="test_track"
resolution = 0.05  
origin = [-10.0, -10.0, 0.0]


os.makedirs(out_path, exist_ok=True)

img=cv2.imread(img_path,cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError("not found image file")

#converting to black and white
_,bw = cv2.threshold(img,200,255,cv2.THRESH_BINARY)

# Invert image (Gazebo expects obstacles = black)
bw = cv2.bitwise_not(bw)
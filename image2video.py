import cv2
import numpy as np
import glob
import os

direct = 'test_videos/dover_road_1/'
size = []
for filename in os.listdir(direct):
    if len(filename) >=22 or len(filename) <=24:
        if len(filename) == 23:
            print("adding 0")
            print(filename)
            newname = filename[:11] + "0" + filename[11:]
            print(newname)
            os.rename(direct + filename, direct + newname)
        if len(filename) == 22:
            print("adding 0")
            print(filename)
            newname = filename[:11] + "00" + filename[11:]
            print(newname)
            os.rename(direct + filename, direct + newname)
        if len(filename) == 21:
            print("adding 0")
            print(filename)
            newname = filename[:11] + "000" + filename[11:]
            print(newname)
            os.rename(direct + filename, direct + newname)
    else:
    	print("found anomaly")
    	break
    
out = cv2.VideoWriter('project2.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 25, (1280,720), True)
for filename in sorted(os.listdir(direct)):
    if filename.endswith("jpg"):
        print(filename)
        img = cv2.imread(direct + filename)
        out.write(img)

out.release()

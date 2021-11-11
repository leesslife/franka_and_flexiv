from argparse import ArgumentParser
import numpy as np
from time import sleep
import sys
import cv2
#sys.path.append("../build")
sys.path.append("../")
from cam import CameraL
from robot import Panda


if __name__=="__main__":
    cam=CameraL()
    num_record=0
    panda=Panda()
    while(True):
        color,depth = cam.get_data()
        cv2.imshow('vis', color)
        action=cv2.waitKey(1)
        if action & 0xFF == ord('q'):
            cv2.destroyAllWindows() 
            break
        if action & 0xFF == ord('s'):
            print("saving the {}-th data".format(num_record))
            current_pose = np.array(panda.robot.read_once().O_T_EE).reshape(4, 4).T
            # time.sleep(1)
            np.save('./franka/H_{}.npy'.format(num_record), current_pose)
            cv2.imwrite('./franka/{}.jpg'.format(num_record), color)
            num_record+=1
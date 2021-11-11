from argparse import ArgumentParser
import numpy as np
from time import sleep
import sys
import sys
#sys.path.append("../build")
sys.path.append("../")
from rfrankx import Robot


import os 
import sys
sys.path.append("..")
#from control_api import batch_quat_to_mat

import numpy as np
from time import sleep

#from pose import batch_quat_to_mat   #
#from control_api import *

import pyrealsense2 as rs
import numpy as np
import cv2


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 1280,720, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8, 30)



# Start streaming
pipeline.start(config)

def pause():
    print("Press Enter to continue...")
    input()

'''
def get_pos_rot_from_xyzq(xyzq):
    pos = np.array([xyzq[0], xyzq[1], xyzq[2]])
    rot = batch_quat_to_mat([[xyzq[3],xyzq[4], xyzq[5], xyzq[6]]])[0]
    return pos, rot 
'''
def get_img():
    frames = pipeline.wait_for_frames()
    # depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())    
    return color_image


def main():
    num_record = 0
    deg_to_rad = lambda x: x / 180. * np.pi
    #flexiv = FlexivRobot("192.168.2.100","192.168.2.110")
    #print('Robot On: {}'.format(flexiv.emergency_state))
    #print('Is Moving: {}'.format(flexiv.is_moving()))
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    robot = Robot(args.host)
    robot.set_default_behavior()

    while(True):
        img = get_img()
        cv2.imshow('vis', img)
        action=cv2.waitKey(1)
        if action & 0xFF == ord('q'):
            cv2.destroyAllWindows() 
            break
        if action & 0xFF == ord('s'):
            print("saving the {}-th data".format(num_record))
            state = robot.read_once()
            print("\O_T_EE",state.O_T_EE,type(state.O_T_EE))
            pose=np.array(state.O_T_EE).reshape(4,4).T
            print(pose)
            #pos, rot = get_pos_rot_from_xyzq(flexiv.tcp_pose)
            np.save('./save_calibration/H_{}.npy'.format(num_record), pose)
            #np.save('./save_calibration/r_{}.npy'.format(num_record), rot)
            cv2.imwrite('./save_calibration/{}.jpg'.format(num_record), img)
            num_record += 1
            #cv2.destroyAllWindows()
            #break

    pipeline.stop()

if __name__ == '__main__':
    main()
#!/usr/bin/python3.6
"""
A script used to collect robot arm poses and realsense images for hand eye calibration.
Move the arm to different poses and make sure the markers can be seen from the camera, 
then press 's' to save the data. Generally, 15-20 datas are enough for calibration.
"""
import os 
import sys
sys.path.append("../..")

from control_api import batch_quat_to_mat, flexiv_robot, pose

import numpy as np
from time import sleep

#from pose import batch_quat_to_mat   #
from control_api import *

import pyrealsense2 as rs
import numpy as np
import cv2
current_pose=np.array([0.1,0.1,0.1,0.1,0.1,0.1,0.1])


#for robot online move
from pynput import keyboard
import threading


class ControlThread(threading.Thread):
    def __init__(self,threadID,name,counter):
        threading.Thread.__init__(self)
        self.threadID=threadID
        self.name=name
        self.counter=counter
        

    def run(self):
        global sf
        def on_press(key):
            quat=np.array([current_pose[3],current_pose[4],current_pose[5],current_pose[6]])
            #print(quat)
            euler=quat_to_euler(quat)
            #quat2=RPY2Quar(euler)

            #print(euler)
            #print(quat2)
            if 'Key.up'==str(key):
                current_pose[0]+=0.0005
                print(current_pose)
            if 'Key.down'==str(key):
                current_pose[0]-=0.0005
                print(current_pose)
            if 'Key.left'==str(key):
                current_pose[1]+=0.0005
                print(current_pose)
            if 'Key.right'==str(key):
                current_pose[1]-=0.0005
                print(current_pose)
            if "'e'"==str(key):
                current_pose[2]+=0.0005
                print(current_pose)
            if "'d'"==str(key):
                current_pose[2]-=0.0005
                print(current_pose)
            if "'r'"==str(key):
                euler[0]+=0.001
                current_pose[3:]=RPY2Quar(euler)
                print(current_pose)
            if "'t'"==str(key):
                euler[0]-=0.001
                current_pose[3:]=RPY2Quar(euler)
                print(current_pose)
            if "'f'"==str(key):
                euler[1]+=0.001
                current_pose[3:]=RPY2Quar(euler)
                print(current_pose)
            if "'g'"==str(key):
                euler[1]-=0.001
                current_pose[3:]=RPY2Quar(euler)
                print(current_pose)
            if "'v'"==str(key):
                euler[2]+=0.001
                current_pose[3:]=RPY2Quar(euler)
                print(current_pose)
            if "'b'"==str(key):
                euler[2]-=0.001
                current_pose[3:]=RPY2Quar(euler)
                print(current_pose)
            #if "'s'"==str(key):
            #    sf=1
             #   print(current_pose)
            #print(current_pose)


        def on_release(key):
            pass

        print("开始控制机器人")
        # Collect events until released
        with keyboard.Listener(on_press=on_press,
                                on_release=on_release) as listener:
            listener.join()
        

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


# Start streaming

pipeline.start(config)
#device = profile.get_device()
#depth_sensor = device.first_depth_sensor()
#device.hardware_reset() 

def pause():
    print("Press Enter to continue...")
    input()

def get_pos_rot_from_xyzq(xyzq):
    pos = np.array([xyzq[0], xyzq[1], xyzq[2]])     #x,y,z
    rot = batch_quat_to_mat(np.array([[xyzq[3],xyzq[4], xyzq[5], xyzq[6]]]))[0]   # w,x,y,z
    return pos, rot 

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
    flexiv = FlexivRobot("192.168.2.100","192.168.2.110")
    #print('Robot On: {}'.format(flexiv.emergency_state))
    print('Is Moving: {}'.format(flexiv.is_moving()))
    print(flexiv.get_tcp_pose())
    global current_pose
    current_pose=flexiv.get_tcp_pose()

    thread=ControlThread(1,"Thread-1",1)
    thread.start()

    #while(True):
    #    sleep(0.01)
    #    flexiv.send_online_pose(current_pose)
    
    while(True):
        img = get_img()
        cv2.imshow('vis', img)
        flexiv.send_online_pose(current_pose)
        action=cv2.waitKey(1)
        if action == ord('q'):
            cv2.destroyAllWindows()
            break
        elif action == ord('s'):
            print("saving the {}-th data".format(num_record))
            pos, rot = get_pos_rot_from_xyzq(flexiv.get_tcp_pose())
            print("pos::",pos)     # pos 是个位置向量
            print("rot::",rot)     # rot 是旋转矩阵
            print("quat::",mat_to_quat(rot))
            print("tcp_pose::",flexiv.get_tcp_pose())
            np.save('./flexiv/t_{}.npy'.format(num_record), pos)
            np.save('./flexiv/r_{}.npy'.format(num_record), rot)
            cv2.imwrite('./flexiv/{}.jpg'.format(num_record), img)
            num_record += 1

    pipeline.stop()
  

if __name__ == '__main__':
    main()

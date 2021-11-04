#!/usr/bin/python3.6
from time import sleep
import cv2
import sys
sys.path.append("/home/franka/home/rflexivx/")

from control_api import *
import numpy as np

def main():
    '''
    connect robot:
    Args:FlexivRobot(robot_ip_address,pc_ip_address)
            robot_ip_address: robot_ip address string
            pc_ip_address: pc_ip address string
    '''
    client=FlexivRobot("192.168.2.100","192.168.2.200")


    '''
    read_cartesian_position  rad
    get_tcp_pos:return np.array of 7D current Cartesian position value(x,y,z,rw,rx,ry,rz)
    
    '''
    print(client.get_tcp_pose())

    '''
    move robot to Cartesian pos
    
    Args:
            max_jnt_vel: maximum joint velocity for each joint, 7d array or list
                       : Default value [6, 6, 7, 7, 14, 14, 14],
            max_jnt_acc: maximum joint acceleration for each joint, 7d array or list
                       : [3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40]
            target: target tcp coordinate, 7d array or list(x,y,z,rw,rx,ry,rz)

            waypoints: array list of waypoints between initial and target poses, e.g. [[0,0,0,0,0,0,0]]
            prefer_jnt_pos: preferred target joint configuration. The robot is compliance with Cartesian constraint while reaching this configuration as close as possible.
                          : Default value[0.0, -40.0, 0.0, 90.0, 0.0, 40.0, 0.0],
            target_tol_level: tolerance level of target pose accuracy ranged from 1 to 10, where 1 is the smallest tolerence

        Returns: None

        """
    '''

    array1=np.array([ 0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742])
    client.move_ptp(array1, 
                    max_jnt_vel=[6, 6, 7, 7, 14, 14, 14],
                    max_jnt_acc=[3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40])


    array2=np.array([ 0.5518,0.211,0.607,0.1675,-0.237,0.94,0.1742])
    client.move_ptp(array2, 
                    max_jnt_vel=[24, 24, 28, 28, 56, 56, 56],
                    max_jnt_acc=[3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40])

    array3=np.array([ 0.4518,0.411,0.307,0.1675,-0.237,0.94,0.1742])
    client.move_ptp(array3, 
                    max_jnt_vel=[12, 12, 14, 14, 28, 28, 28],
                    max_jnt_acc=[3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40])

    

if __name__=="__main__":
    main()
#!/usr/bin/python3.8
import sys
import numpy as np
import cv2

from realSceneApi.saveImage.realsense import realsense

class GetACamPara(object):
    def __init__(self):
        self.cam=realsense()
        self.filter_holefilling = self.cam.filterHoleFilling(1)
        self.filter_spatial = self.cam.filterSpatial(5,0.25,50)
        self.filter_decimation = self.cam.filterDecimation(3)
        self.cam.start(resolution='720p',depth_preset=3,color_scheme=0,histogram_equalization=True,enable_ir_emitter=1)

    def getIntrInfo(self,robot_name="Flexiv"):
        if robot_name=="Flexiv":
            color_intr_flexiv,depth_intr_flexiv,depth_to_color_extr_flexiv,cam_para_flexiv=self.cam.getCamPara(robot_name="Flexiv")
            return color_intr_flexiv,depth_intr_flexiv
        elif robot_name=="Franka":
            color_intr_franka,depth_intr_franka,depth_to_color_extr_franka,cam_para_franka=self.cam.getCamPara(robot_name="Franka")
            return color_intr_franka,depth_intr_franka
        else:
            return None

    def getImage(self,robot_name="Flexiv"):
        if robot_name=="Flexiv":
            color_img_flexiv, depth_map_flexiv, depth_img_flexiv=self.cam.numpyFrame(self.filter_decimation,self.filter_spatial,colorizer=None,align=None,robot_name="Flexiv")
            return color_img_flexiv, depth_map_flexiv, depth_img_flexiv
        elif robot_name=="Franka":
            color_img_franka, depth_map_franka, depth_img_franka=self.cam.numpyFrame(self.filter_decimation,self.filter_spatial,colorizer=None,align=None,robot_name="Franka")
            return color_img_franka, depth_map_franka, depth_img_franka
        else:
            return None
    
    def stop(self):
        self.cam.stop()

class getHInfo(object):
    def __init__(self):
        self.flexivH2E=np.load('./realSceneApi/calibration/realtcab/save/flexiv2franka/flexivH2E.npy')
        self.frankaH2E=np.load('./realSceneApi/calibration/realtcab/save/flexiv2franka/frankaH2E.npy')
        self.franka2GoalH=np.load('./realSceneApi/calibration/realtcab/save/flexiv2franka/franka2GoalH.npy')
        self.flexiv2GoalH=np.load('./realSceneApi/calibration/realtcab/save/flexiv2franka/flexiv2GoalH.npy')
        self.franka2flexiv=np.load('./realSceneApi/calibration/realtcab/save/flexiv2franka/franka2flexiv.npy')
    
    '''
    index=0 :get flexivH2E
    index=1 :get frankaH2E
    index=2 :get franka2GoalH
    index=3 :get flexiv2GoalH
    index=4 :get franka2flexiv
    '''
    def getH(self,index=0):
        if index==0:
            return self.flexivH2E
        elif index==1:
            return self.frankaH2E
        elif index==2:
            return self.franka2GoalH
        elif index==3:
            return self.franka2GoalH
        elif index==4:
            return self.franka2flexiv
        else:
            return None


if __name__=="__main__":
    '''
    camInfo=GetACamPara()

    while 1:
        key = cv2.waitKey(1) & 0xFF    
        color_img_flexiv, depth_map_flexiv, depth_img_flexiv=camInfo.getImage(robot_name="Flexiv")
        color_img_franka, depth_map_franka, depth_img_franka=camInfo.getImage(robot_name="Franka")
        color_intr_flexiv,depth_intr_flexiv=camInfo.getIntrInfo(robot_name="Flexiv")
        color_intr_franka,depth_intr_franka=camInfo.getIntrInfo(robot_name="Franka")
        print(color_intr_flexiv.fx)
        print(depth_intr_flexiv.fx)
        print(color_intr_franka.fx)
        print(depth_intr_franka.fx)
        color_depth_flexiv=np.hstack((color_img_flexiv, depth_img_flexiv))
        color_depth_franka=np.hstack((color_img_franka, depth_img_franka))
        color_depth=np.vstack((color_depth_flexiv,color_depth_franka))
        cv2.imshow("test", color_depth)
                
        if key== 27: # esc
            break
        '''
    H=getHInfo()
    print("flexivH2E",H.getH(index=0))
    print("frankaH2E",H.getH(index=1))
    print("franka2GoalH",H.getH(index=2))
    print("franka2GoalH",H.getH(index=3))
    print("franka2flexiv",H.getH(index=4))
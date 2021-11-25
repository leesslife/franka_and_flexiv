#!/usr/bin/python3.8
from posixpath import join
import sys
import numpy as np
import cv2
from time import sleep
import paho.mqtt.client as mqtt
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

class DControlApi(object):
    def __init__(self):
        self.client=mqtt.Client()
        self.client.connect('127.0.0.1',1883,100)
        self.client.publish("franka_dynamix_rel",payload=0.15,qos=0)
    
    def flexivJointControl(self,JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],
                                VecList=[18,18,21,21,42,42,42],
                                AccList=[10.8,10.8,12.6,12.6,25.2,25.2,25.2],
                                SigFlag=False,
                                SigId="None"):
        jointcmd='['+','.join(str(i) for i in JointList)+']'
        veccmd='['+','.join(str(i) for i in VecList)+']'
        acccmd='['+','.join(str(i) for i in AccList)+']'
        cmd="jointmotion="+jointcmd+'|'+"max_jnt_vel="+veccmd+'|'+'max_jnt_acc='+acccmd
        if SigFlag==False:
            #print(cmd)
            self.client.publish('flexiv_joint_motion',payload=cmd,qos=0)
        if SigFlag==True:
            cmd=cmd+'@@'+SigId
            #print(cmd)
            self.client.publish('flexiv_joint_motion',payload=cmd,qos=0)
    
    # 抓手行为:1表示抓紧，0表示释放，
    # gripper_speed :抓手速度0-255 之间
    # gripper_forced:抓手力矩0-255 之间
    def flexivGrasp(self,gripper_speed=100,gripper_forced=100,clamp=0,SigFlag=False,SigId="None"):
        gripperscmd="gripper_speed=["+str(gripper_speed)+"]"
        gripperfcmd="gripper_forced=["+str(gripper_forced)+"]"
        clampcmd="clamp=["+str(clamp)+"]"
        cmd=gripperscmd+"__"+gripperfcmd+"__"+clampcmd
        if SigFlag==False:
            #print(cmd)
            self.client.publish('flexiv_grasp',payload=cmd,qos=0)
        if SigFlag==True:
            cmd=cmd+'@@'+SigId
            self.client.publish('flexiv_grasp',payload=cmd,qos=0)
            #print(cmd)
    
    # 笛卡尔坐标系 x,y,z, Nw,Nx,Ny,Nz
    def flexivptpmotion(self,ptpmotion=[0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742],
                             max_jnt_vel=[18,18,21,21,42,42,42],
                             max_jnt_acc=[10.8,10.8,12.6,12.6,25.2,25.2,25.2],
                             SigFlag=False,
                             SigId="None"):
        ptpcmd='['+','.join(str(i) for i in ptpmotion)+']'
        veccmd='['+','.join(str(i) for i in max_jnt_vel)+']'
        acccmd='['+','.join(str(i) for i in max_jnt_acc)+']'
        cmd="ptpmotion="+ptpcmd+"|"+"max_jnt_vel="+veccmd+"|"+"max_jnt_acc="+acccmd
        if SigFlag==False:
            #print(cmd)
            self.client.publish('flexiv_ptp_motion',payload=cmd,qos=0)
        if SigFlag==True:
            cmd=cmd+'@@'+SigId
            #print(cmd)
            self.client.publish('flexiv_ptp_motion',payload=cmd,qos=0)

    # 分别读取笛卡尔坐标系与节点空间值
    # [1,0.5,flexiv_J_save1]
    # 1：表示读取数据的量，如果负数则表示无限读取
    # 0.5：表示读取间隔
    # flexiv_J_save1 表示去读文件的值
    # flexiv可以异步读取数据，可以设置读取数据量为-1
    def flexivreadjoint(self,readnum=1,readinterval=0.5,flexiv_J_savefile="flexiv_J_save1"):
        cmd="["+str(readnum)+","+str(readinterval)+","+flexiv_J_savefile+"]"
        #print(cmd)
        self.client.publish('read_flexiv_joint',payload=cmd,qos=0)
    
    def flexivreadcartesian(self,readnum=1,readinterval=0.5,flexiv_P_savefile="flexiv_P_save1"):
        cmd="["+str(readnum)+","+str(readinterval)+","+flexiv_P_savefile+"]"
        #print(cmd)
        self.client.publish('read_flexiv_cartesian',payload=cmd,qos=0)

    # max_translation_velocity {1.7}; // [m/s]
    # max_rotation_velocity {2.5}; // [rad/s]
    # max_elbow_velocity {2.175}; // [rad/s]
    # max_translation_acceleration {13.0}; // [m/s²]
    # max_rotation_acceleration {25.0}; // [rad/s²]
    # max_elbow_acceleration {10.0}; // [rad/s²]
    # max_translation_jerk {6500.0}; // [m/s³]
    # max_rotation_jerk {12500.0}; // [rad/s³]
    # max_elbow_jerk {5000.0}; // [rad/s³]
    # robot.set_dynamic_rel(0.15) this factor times max_value 1.7x0.15x0.4
    def frankaJointControl(self,JointList=[0.1,-0.785398163397448279,0.1,-2.356194490192344837,0.0,1.570796326794896558, 0.785398163397448279],
                                dynamix_rel=0.15,SigFlag=False,SigId="None"):
        jointcmd='jointmotion=['+','.join(str(i) for i in JointList)+']'
        dynamixcmd='dynamix_rel='+str(dynamix_rel)
        cmd=jointcmd+"|"+dynamixcmd
        if SigFlag==False:
            print(cmd)
        else:
            cmd=cmd+"@@"+SigId
            print(cmd)
    
    def setFrankaDynamicRel(self,dynamic_rel=0.15):
        self.client.publish("franka_dynamix_rel",payload=dynamic_rel,qos=0)

    # control_mode=Absolute ||control_mode=Relative
    def frankarfwaypoint(self,rfwaypoint_list=[{"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},],
                             SigFlag=False,SigId="None"):
        rfwaypointcmd=""
        for i in range(len(rfwaypoint_list)):
            if i!=len(rfwaypoint_list)-1:
                waypoint="rfwaypoint_trans=["+",".join(str(j) for j in rfwaypoint_list[i]["rfwaypoint_trans"])+"]__" \
                         "rfwaypoint_R=["+",".join(str(j) for j in rfwaypoint_list[i]["rfwaypoint_R"])+"]__" \
                         "control_mode="+rfwaypoint_list[i]["control_mode"]
                rfwaypointcmd=rfwaypointcmd+waypoint+"|"
            else:
                waypoint="rfwaypoint_trans=["+",".join(str(j) for j in rfwaypoint_list[i]["rfwaypoint_trans"])+"]__" \
                         "rfwaypoint_R=["+",".join(str(j) for j in rfwaypoint_list[i]["rfwaypoint_R"])+"]__" \
                         "control_mode="+rfwaypoint_list[i]["control_mode"]
                rfwaypointcmd=rfwaypointcmd+waypoint
        
        if SigFlag==False:
            print(rfwaypointcmd)
        else:
            rfwaypointcmd=rfwaypointcmd+"@@"+SigId
            print(rfwaypointcmd)
            


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
    '''
    H=getHInfo()
    print("flexivH2E",H.getH(index=0))
    print("frankaH2E",H.getH(index=1))
    print("franka2GoalH",H.getH(index=2))
    print("franka2GoalH",H.getH(index=3))
    print("franka2flexiv",H.getH(index=4))

    dc=DControlApi()
    dc.frankaJointControl()
    '''

    dcontrl=DControlApi()
    '''
    #dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077])
    #dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,0.619,1.62,0.001596,0.5288,0.004077])
    #dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077])
    #dcontrl.flexivGrasp(clamp=1)
    #dcontrl.flexivptpmotion(ptpmotion=[0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742])
    #dcontrl.flexivGrasp(clamp=0)
    #dcontrl.flexivreadjoint()
    #dcontrl.flexivreadcartesian()
    '''
    
    dcontrl.frankaJointControl()
    dcontrl.frankarfwaypoint()
    
    print("=========================================================")

    dcontrl.frankaJointControl(JointList=[0.1, -0.385398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279],dynamix_rel=0.2,SigFlag=True,SigId="V2")
    dcontrl.setFrankaDynamicRel(dynamic_rel=0.1)
    dcontrl.frankarfwaypoint(rfwaypoint_list=[{"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                              {"rfwaypoint_trans":[0.4,0.0,0.48],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                              {"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"}],
                                              SigFlag=True,SigId="franka1")
    
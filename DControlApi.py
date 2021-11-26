#!/usr/bin/python3.8
from posixpath import join
import sys
from typing import Counter
import numpy as np
import cv2
from time import sleep
import paho.mqtt.client as mqtt
import os
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
    def Dcab(self):
        flexivCabcmd="realSceneApi/calibration/realtcab/./cab_flexiv.py"
        pfleixv=os.system(flexivCabcmd)
        if pfleixv==0:
            print("flexiv cab is success!")
    
        frankaCabcmd="realSceneApi/calibration/realtcab/./cab_franka.py"
        pfranka=os.system(frankaCabcmd)
        if pfranka==0:
            print("franka cab is success!")
        
        franka2flexivcmd="realSceneApi/calibration/realtcab/./cab_franka2flexiv.py"
        pff=os.system(franka2flexivcmd)
        if pff==0:
            print("Double robot cab is success!")
    '''
        
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
        def on_connect(client, userdata, flags, rc):
            print("Connected with result code: " + str(rc))
        self.client.on_connect=on_connect

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
            #print(cmd)
            self.client.publish('franka_joint_motion',payload=cmd,qos=0)
        else:
            cmd=cmd+"@@"+SigId
            self.client.publish('franka_joint_motion',payload=cmd,qos=0)
            #print(cmd)
    
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
            #print(rfwaypointcmd)
            self.client.publish('franka_rfwaypoint_motion',payload=rfwaypointcmd,qos=0)
        else:
            rfwaypointcmd=rfwaypointcmd+"@@"+SigId
            self.client.publish('franka_rfwaypoint_motion',payload=rfwaypointcmd,qos=0)
            #print(rfwaypointcmd)
            
    # 抓手行为:1表示抓紧，0表示释放，     
    # gripper_speed:m/s
    # gripper_forced:20N/s
    def frankaGrasp(self,gripper_speed=0.02,gripper_forced=20,clamp=0,SigFlag=False,SigId="None"):
        gripperscmd="gripper_speed=["+str(gripper_speed)+"]"
        gripperfcmd="gripper_forced=["+str(gripper_forced)+"]"
        clampcmd="clamp=["+str(clamp)+"]"
        cmd=gripperscmd+"__"+gripperfcmd+"__"+clampcmd
        if SigFlag==False:
            #print(cmd)
            self.client.publish('franka_grasp',payload=cmd,qos=0)
        if SigFlag==True:
            cmd=cmd+'@@'+SigId
            self.client.publish('franka_grasp',payload=cmd,qos=0)

    # 分别读取笛卡尔坐标系与节点空间值
    # [1,0.5,franka_J_save1]
    # 1：表示读取数据的量，如果负数则表示无限读取
    # 0.5：表示读取间隔
    # franka_J_save1 表示去读文件的值
    # franka不可以异步读取数据，设置读取数据量为-1 表示无限读取，会卡死franka接下去的行为，所以尽量不要设置为-1
    def frankareadjoint(self,readnum=1,readinterval=0.5,franka_J_savefile="franka_J_save1"):
        cmd="["+str(readnum)+","+str(readinterval)+","+franka_J_savefile+"]"
        #print(cmd)
        self.client.publish('read_franka_joint',payload=cmd,qos=0)
    
    def frankareadcartesian(self,readnum=1,readinterval=0.5,franka_P_savefile="franka_P_save1"):
        cmd="["+str(readnum)+","+str(readinterval)+","+franka_P_savefile+"]"
        #print(cmd)
        self.client.publish('read_franka_cartesian',payload=cmd,qos=0)
    
    def selfloop(self):
        self.client.loop_forever()

    def Dcallback(self,idl=[]):
        for i in range(len(idl)):
            self.client.subscribe(idl[i],qos=0)


if __name__=="__main__":
    # 获取内参矩阵
    '''
    camInfo=GetACamPara()

    while 1:
        key = cv2.waitKey(1) & 0xFF  
        # 深度图:depth_img 
        # RGB图:color_img 
        # 深度图内参矩阵
        # RGB内参矩阵
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
    # 获取标定结果
    # 标定流程
        # 运行realScenceApi/realtcab中的 cab_flexiv.py
        # 运行realScenceApt/realtcat中的 cab_franka.py
        # 运行realScenceApt/realtcat中的 cab_franka2flexiv.py
        # 运行完成后，通过以下代码就可以读出相应的矩阵的信息了
    H=getHInfo()
    # H2E 手眼矩阵
    print("flexivH2E",H.getH(index=0))
    print("frankaH2E",H.getH(index=1))
    # franka2GoalH 手臂底座到目标
    print("franka2GoalH",H.getH(index=2))
    print("franka2GoalH",H.getH(index=3))
    # franka2flexiv 双手臂底座之间的距离
    print("franka2flexiv",H.getH(index=4))
    '''
 

    # 循环控制案例
 
    dcontrl=DControlApi()
    flexivcout=0
    frankacout=0
    allcout=0
    def on_messaage(client,userdata,msg):
        global flexivcout
        global frankacout
        global allcout
        if msg.topic=="flexiv1":
            if frankacout<2:
                print("=============================Contrl Franka=================================")
                dcontrl.frankaJointControl()
                dcontrl.frankarfwaypoint()
                dcontrl.frankaGrasp(clamp=1)
                dcontrl.frankareadjoint()
    
                dcontrl.frankaJointControl(JointList=[0.1, -0.785398163397448279, 0.1, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279],dynamix_rel=0.2,SigFlag=True,SigId="V2")
                dcontrl.frankaGrasp(clamp=0)
                dcontrl.setFrankaDynamicRel(dynamic_rel=0.2)
                dcontrl.frankarfwaypoint(rfwaypoint_list=[{"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                                        {"rfwaypoint_trans":[0.4,0.0,0.48],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                                        {"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"}],
                                                            SigFlag=True,SigId="franka1")
                frankacout+=1
            else:
                print("=============================Contrl Franka=================================")
                dcontrl.frankaJointControl()
                dcontrl.frankarfwaypoint()
                dcontrl.frankaGrasp(clamp=1)
                dcontrl.frankareadjoint()
    
                dcontrl.frankaJointControl(JointList=[0.1, -0.785398163397448279, 0.1, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279],dynamix_rel=0.2,SigFlag=True,SigId="V2")
                dcontrl.frankaGrasp(clamp=0)
                dcontrl.setFrankaDynamicRel(dynamic_rel=0.2)
                dcontrl.frankarfwaypoint(rfwaypoint_list=[{"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                                        {"rfwaypoint_trans":[0.4,0.0,0.48],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                                        {"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"}],
                                                            SigFlag=True,SigId="franka2")
                dcontrl.frankarfwaypoint(rfwaypoint_list=[ {"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"}],
                                                            SigFlag=True,SigId="franka1")           
                frankacout=0
        
        if msg.topic=="franka1":
            if flexivcout<2:
                print("=============================Contrl Flexiv=================================")
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,0.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivGrasp(clamp=1)
                dcontrl.flexivptpmotion(ptpmotion=[0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742])
                dcontrl.flexivGrasp(clamp=0,SigFlag=True,SigId="flexiv1")
                dcontrl.flexivreadjoint()
                dcontrl.flexivreadcartesian()
                flexivcout+=1
            else:
                print("=============================Contrl Flexiv=================================")
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,0.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivGrasp(clamp=1)
                dcontrl.flexivptpmotion(ptpmotion=[0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742])
                dcontrl.flexivGrasp(clamp=0,SigFlag=True,SigId="flexiv2")
                dcontrl.flexivreadjoint()
                dcontrl.flexivreadcartesian()
                flexivcout=0
        
        if msg.topic=="franka2" or msg.topic=="flexiv2":
            if allcout<1:
                allcout+=1
                print("===========================",allcout)
            else:
                print("===========================",allcout)
                print("=============================Contrl Franka=================================")
                dcontrl.frankaJointControl()
                dcontrl.frankarfwaypoint()
                dcontrl.frankaGrasp(clamp=1)
                dcontrl.frankareadjoint()
    
                dcontrl.frankaJointControl(JointList=[0.1, -0.785398163397448279, 0.1, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279],dynamix_rel=0.2,SigFlag=True,SigId="V2")
                dcontrl.frankaGrasp(clamp=0)
                dcontrl.setFrankaDynamicRel(dynamic_rel=0.2)
                dcontrl.frankarfwaypoint(rfwaypoint_list=[{"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                                        {"rfwaypoint_trans":[0.4,0.0,0.48],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"},
                                                        {"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,0.0],"control_mode":"Absolute"}],)
                frankacout=0
                print("=============================Contrl Flexiv=================================")
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,0.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],VecList=[27, 27, 27, 31, 53, 53, 53],AccList=[15.8, 15.8, 19.6, 19.6, 35.2, 35.2, 35.2])
                dcontrl.flexivGrasp(clamp=1)
                dcontrl.flexivptpmotion(ptpmotion=[0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742])
                dcontrl.flexivGrasp(clamp=0)
                dcontrl.flexivreadjoint()
                dcontrl.flexivreadcartesian()

    # 定义信号接收内容
    dcontrl.Dcallback(idl=["flexiv1","franka1","flexiv2","franka2"])
    # 定义信号接收器
    dcontrl.client.on_message=on_messaage

    print("=============================Contrl Flexiv=================================")
    # 节点控制，可以根据要求改变节点速度与加速度
    dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077])
    dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,0.619,1.62,0.001596,0.5288,0.004077])
    dcontrl.flexivJointControl(JointList=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077])
    # 抓手
    dcontrl.flexivGrasp(clamp=1)
    # ptp坐标
    dcontrl.flexivptpmotion(ptpmotion=[0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742])
    # 抓手，最后设置动作完成发送的信号
    dcontrl.flexivGrasp(clamp=0,SigFlag=True,SigId="flexiv1")
    dcontrl.flexivreadjoint()
    dcontrl.flexivreadcartesian()

 
    

    dcontrl.selfloop()
    
    
    
    
    
    
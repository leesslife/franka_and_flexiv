#!/usr/bin/python3.8
from os import path
import sys
from typing import ValuesView, cast
from numpy.core.fromnumeric import take
sys.path.append("/home/franka/home/rfrankx/build")
from rfrankx import JointMotion,Robot,Waypoint,WaypointMotion,Affine,MoveWaypointMotion
from argparse import ArgumentParser
import paho.mqtt.client as mqtt 
from time import sleep
import numpy as np

class RobotControl(object):
    def __init__(self):
        self.parser=ArgumentParser()
        self.parser.add_argument('--host',default='172.16.0.2',help='FCI IP of the robot')
        self.args=self.parser.parse_args()

        #Connect to the robot
        self.robot_init()

        #Robot state
        #0 is stop
        #1 is runing
        self.state=0
        self.taskqueque=[]
        self.dynamic_rel=[0.15]
    
    def robot_init(self):
        #Connect to the robot
        self.robot =Robot(self.args.host)
        self.robot.set_default_behavior()
        self.robot.recover_from_errors()
        self.robot.set_dynamic_rel(0.15)
    
    def exit_error_info(self,info):
        self.state=0
        print(info)
        try:
            self.taskqueque.pop(0)
        except Exception as e:
            print("taskqueque is empty!")
        return 

    def get_motion_value(self,pathlist,pathlen):
        index_L=pathlist.find("[")
        index_R=pathlist.find("]")
        if (index_L==-1) and (index_R==-1):
            self.exit_error_info("cant find '[' or ']' set!")
            return -9999
        elif (index_L==-1) and (index_R!=-1):
            self.exit_error_info("cant find '['!")
            return -9999
        elif (index_L!=-1) and (index_R==-1):
            self.exit_error_info("cant find ']'!")
            return -9999
        else:
            if index_L>=index_R:
                self.exit_error_info("'[' and ']' set wrong!")
                return -9999
            else:
                motionString=pathlist[index_L+1:index_R].replace(" ","")
                motionString=''.join([i for i in motionString if not i.isalpha()])
                try:
                    Value=list(map(float,motionString.split(','))) # get Value
                except Exception as e:
                    print("Vaule can`t be converted!")
                    return -9999
                if len(Value)!=pathlen:
                    self.exit_error_info("the length of cmd data is wrong!")
                    return -9999
                else:
                    return Value

    def op_franka_joint_motion(self):
        payloadstring=self.taskqueque[0].payload.decode()
        indexJ=payloadstring.find("jointmotion")
        indexRel=payloadstring.find("dynamic_rel")
        joint_motionMap={"joint_motion":[0.1, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279],
                         "dynamic_rel":0.15}

        if indexJ==-1 and indexRel==-1:
            self.exit_error_info("no jointmotion set value!")
            return 
        elif indexJ!=-1 and indexRel==-1:
            joint_motionMap["dynamic_rel"]=0.15
            jointcmdlist=payloadstring.split("|")
            for i in range(len(jointcmdlist)):
                indexJJ=jointcmdlist[i].find("jointmotion")
                if indexJJ!=-1:
                    Value=self.get_motion_value(jointcmdlist[i],7)
                    joint_motionMap["joint_motion"]=Value
        elif indexJ==-1 and indexRel!=-1:
            self.exit_error_info("no jointmotion pos value!")
            return
        else:
            jointcmdlist=payloadstring.split("|")
            for i in range(len(jointcmdlist)):
                indexJJ=jointcmdlist[i].find("jointmotion")
                indexRRel=jointcmdlist[i].find("dynamic_rel")
                if indexJJ==-1 and indexRRel==-1:
                    self.exit_error_info("no jointmotion and dynamic_rel set value!")
                    return 
                elif indexJJ!=-1 and indexRRel!=-1:
                    self.exit_error_info("check your cmd for some wrong!")
                    return 
                elif indexJJ!=-1 and indexRRel==-1:
                    Value=self.get_motion_value(jointcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("Franka Joint Value is wrong,breakup!")
                        return
                    joint_motionMap["joint_motion"]=Value
                else:
                    Value=self.get_motion_value(jointcmdlist[i],1)
                    if Value==-9999:
                        self.exit_error_info("Franka Joint Value is wrong,breakup!")
                        return
                    joint_motionMap["dynamic_rel"]=Value[0]
        print(joint_motionMap["dynamic_rel"])
        print(joint_motionMap["joint_motion"])
        self.robot.set_dynamic_rel(joint_motionMap["dynamic_rel"])
        joint_motion=JointMotion(joint_motionMap["joint_motion"])
        self.robot.move(joint_motion)
        sleep(0.05)
        try:
            self.taskqueque.pop(0)
        except Exception as e:
            print("franka option is over!")

    def op_franka_waypoints_motion(self):
        payloadstring=self.taskqueque[0].payload.decode()
        pathlist=payloadstring.split('|')
        pathmaplist=[]
        for i in range(len(pathlist)):
            StringWP=pathlist[i]
            indexWP=StringWP.find("waypoint")
            indexDR=StringWP.find("dynamic_rel")
            if (indexWP==-1) and (indexDR==-1):
                self.exit_error_info("cant find dynamic_rel and  waypoint set!")
                return 
            elif (indexDR!=-1) and (indexWP!=-1):
                self.exit_error_info("dynamic_rel and  waypoint at same time!")
                return 
            elif (indexDR==-1) and (indexWP!=-1):
                waypointMap={"waypoint":[0.33,0,0.58,0,0,0],"elbow":[0],"control_mode":"Absolute"}
                cmdlist=StringWP.split('__')
                if len(cmdlist)<=0:
                    self.exit_error_info("no waypoint path set!")
                    return
                else:     
                    for i in range(len(cmdlist)):
                        if i==0:                         
                            index=cmdlist[i].find('waypoint')
                            if index==-1:
                                self.exit_error_info("1 waypoint path set is invalid!")
                                return 
                            else:
                                Value=self.get_motion_value(cmdlist[i],6)
                                if Value==-9999:
                                    self.exit_error_info("Franka waypoint Value is wrong,breakup!")
                                    return
                                waypointMap["waypoint"]=Value
                        if i==1:
                            index=cmdlist[i].find('elbow')
                            if index==-1:
                                waypointMap["elbow"]=[0]
                                print("elbow Value use default Value!")
                            else:
                                Value=self.get_motion_value(cmdlist[i],1)
                                if Value==-9999:
                                    self.exit_error_info("Franka waypoint Value is wrong,breakup!")
                                    return
                                waypointMap["elbow"]=Value         
                                pass

                        if i==2:
                            index=cmdlist[i].find('control_mode')
                            if index==-1:
                                self.exit_error_info("1 waypoint path set is invalid!")
                                return 
                            else:
                                indexCMR=cmdlist[i].find("Relative")
                                indexCMA=cmdlist[i].find("Absolute")
                                if (indexCMA==-1) and (indexCMR==-1):
                                    waypointMap["control_mode"]="Absolute"
                                elif (indexCMA==-1) and (indexCMR!=-1):
                                    waypointMap["control_mode"]="Relative"
                                elif (indexCMA!=-1) and (indexCMR==-1):
                                    waypointMap["control_mode"]="Absolute"
                                else:
                                    waypointMap["control_mode"]="Absolute"
                                pass
                        pathmaplist.append(waypointMap)
            else:
                dynamic_rel_map={"dynamic_rel":0.15}
                index_dynamic_rel=StringWP.find("dynamic_rel")
                if index_dynamic_rel==-1:
                    dynamic_rel_map["dynamic_rel"]=0.15
                else:
                    Value=self.get_motion_value(StringWP,1)
                    if Value==-9999:
                        self.exit_error_info("waypoint value is wrong!")
                        return 
                    dynamic_rel_map["dynamic_rel"]=Value[0]
                pathmaplist.append(dynamic_rel_map)
              
            waypointList=[]
            for j in range(len(pathmaplist)):       
                if 'waypoint' in pathmaplist[j].keys():
                    wayp=pathmaplist[j]["waypoint"]
                    elbow_value=pathmaplist[j]["elbow"][0]
                    affine=Affine(wayp[0],wayp[1],wayp[2],wayp[3],wayp[4],wayp[5])
                    if pathmaplist[j]["control_mode"]=="Relative":
                        waypoint=Waypoint(affine,elbow_value,Waypoint.Relative)
                    if pathmaplist[j]["control_mode"]=="Absolute":
                        waypoint=Waypoint(affine,elbow_value,Waypoint.Absolute)
                    waypointList.append(waypoint)
                if 'dynamic_rel' in pathmaplist[j].keys():
                    self.robot.set_dynamic_rel(pathmaplist[j]['dynamic_rel'])  
        print(pathmaplist)
        motion_down=WaypointMotion(waypointList)
        #self.robot.move(motion_down)  //some wrong for ruig
        try:
            self.taskqueque.pop(0)
        except Exception as e:
            print("franka option is over!") 
    
    def op_franka_rfwaypoints_motion(self):
        payloadstring=self.taskqueque[0].payload.decode()
        pathlist=payloadstring.split('|')
        pathmaplist=[]
        for i in range(len(pathlist)):
            rfwaypointMap={"rfwaypoint_trans":[0.3,0.0,0.58],"rfwaypoint_R":[0.0,3.14,2.5],"control_mode":"Absolute"}
            StringWP=pathlist[i]
            indexT=StringWP.find("rfwaypoint_trans")
            indexR=StringWP.find("rfwaypoint_R")
            indexCM=StringWP.find("control_mode")
            if(indexT==-1) or (indexR==-1):
                self.exit_error_info("cant find rfwaypoint_trans and rfwaypoint_R set!")
                return
            else:
                if indexCM==-1:
                    rfwaypointMap["control_mode"]="Absolute"
                else:
                    cmdlist=StringWP.split("__")
                    #print(cmdlist)
                    for j in range(len(cmdlist)):
                        if j==0:
                            indexTT=cmdlist[j].find("rfwaypoint_trans")
                            if indexTT==-1:
                                self.exit_error_info("cant find rfwaypoint_trans set!")
                                return
                            else:
                                Value=self.get_motion_value(cmdlist[j],3)
                                if Value==-9999:
                                    self.exit_error_info("rfwaypoint_motion is wrong!")
                                    return  
                                rfwaypointMap["rfwaypoint_trans"]=Value

                        if j==1:
                            indexTT=cmdlist[j].find("rfwaypoint_R")
                            if indexTT==-1:
                                self.exit_error_info("cant find rfwaypoint_R set!")       
                                return
                            else:
                                Value=self.get_motion_value(cmdlist[j],3)
                                if Value==-9999:
                                    self.exit_error_info("rfwaypoint_Rmotion is wrong!")
                                    return 
                                rfwaypointMap["rfwaypoint_R"]=Value

                        if j==2:
                            index=cmdlist[j].find('control_mode')
                            if index==-1:
                                self.exit_error_info("control_mode set is invalid!")
                                return 
                            else:
                                indexCMR=cmdlist[j].find("Relative")
                                indexCMA=cmdlist[j].find("Absolute")
                                if (indexCMA==-1) and (indexCMR==-1):
                                    rfwaypointMap["control_mode"]="Absolute"
                                elif (indexCMA==-1) and (indexCMR!=-1):
                                    rfwaypointMap["control_mode"]="Relative"
                                elif (indexCMA!=-1) and (indexCMR==-1):
                                    rfwaypointMap["control_mode"]="Absolute"
                                else:
                                    rfwaypointMap["control_mode"]="Absolute"
                    pathmaplist.append(rfwaypointMap)
        #print(pathmaplist)
        rfwaypointlist=[]
        #print(pathmaplist)
        for i in range(len(pathmaplist)):
            trans=np.array(pathmaplist[i]["rfwaypoint_trans"])
            rot=np.array(pathmaplist[i]["rfwaypoint_R"])
            if pathmaplist[i]['control_mode']=="Absolute":
                rfwaypoint=Waypoint(trans,rot,Waypoint.Absolute)
            else:
                rfwaypoint=Waypoint(trans,rot,Waypoint.Relative)
            rfwaypointlist.append(rfwaypoint)
        #print(rfwaypointlist)
        motion_down_rfwaypoint=MoveWaypointMotion(rfwaypointlist)
        self.robot.move(motion_down_rfwaypoint)
        self.taskqueque.pop(0)
        return 

    def op_franka_franka_grasp(self):
        graspString=self.taskqueque[0].payload.decode()
        graspsetlist=graspString.split("__")
        graspMap={"gripper_speed":0.02,"gripper_forced":20.0,"clamp":0}

        for i in range(len(graspsetlist)):  
            indexGS=graspsetlist[i].find("gripper_speed")
            indexGF=graspsetlist[i].find("gripper_forced")
            indexCL=graspsetlist[i].find("clamp")
            if indexGS!=-1:
                Value=self.get_motion_value(graspsetlist[i],1)
                if Value==-9999:
                    self.exit_error_info("gripper_speed set is wrong!")
                    return 
                graspMap["gripper_speed"]=Value[0]
            if indexGF!=-1:
                Value=self.get_motion_value(graspsetlist[i],1)
                if Value==-9999:
                    self.exit_error_info("gripper_forced set is wrong!")
                    return
                graspMap["gripper_forced"]=Value[0]
            if indexCL!=-1:
                Value=self.get_motion_value(graspsetlist[i],1)
                if Value==-9999:
                    self.exit_error_info("clamp set is wrong!")
                    return
                graspMap["clamp"]=Value[0]
                    
            if indexCL==-1 and indexGF==-1 and indexGS==-1:
                self.exit_error_info("no clamp set!break up!")
                return

        gripper=self.robot.get_gripper()
        gripper.gripper_speed=graspMap["gripper_speed"]
        gripper.gripper_forced=graspMap["gripper_forced"]
        if graspMap["clamp"]==1:
            gripper.clamp()
            self.exit_error_info("clamp")
            return 
        else:
            try:
                gripper.release(3.0)
            except Exception as e: 
                print("no wrong")
            self.exit_error_info("no wrong")
            return 
                       
    def run(self):
        self.state=1
        while len(self.taskqueque)!=0:
            if self.taskqueque[0].topic=='franka_joint_motion':
                self.op_franka_joint_motion()
            elif self.taskqueque[0].topic=="franka_waypoints_motion":
                # Affine(double x, double y, double z, double a = 0.0, double b = 0.0, double c = 0.0)
                self.op_franka_waypoints_motion()
            elif self.taskqueque[0].topic=="franka_rfwaypoint_motion":
                self.op_franka_rfwaypoints_motion()
            elif self.taskqueque[0].topic=="franka_dynamic_rel":
                self.robot.set_dynamic_rel(float(self.taskqueque[0].payload))
                self.taskqueque.pop(0)
            elif self.taskqueque[0].topic=="franka_grasp":
                self.op_franka_franka_grasp()
            else:
                self.state=0
                print("no match control mode set value!")
                self.taskqueque.pop(0)
                return  
        self.state=0

franka_robot=RobotControl()

def on_connect(client,userdata,flasg,rc):
    print("Connected with result code:"+str(rc))

def on_message(client,userdata,msg):
    if franka_robot.state==0:
        franka_robot.taskqueque.append(msg)
        franka_robot.run()
    if franka_robot.state==1:
        franka_robot.taskqueque.append(msg)


class mqttFranka(object):
    def __init__(self,ip,port,interval):
        self.client=mqtt.Client()
        self.client.on_connect=on_connect
        self.client.on_message=on_message
        self.client.connect(ip,port,interval)
    
    def sub(self,topic):
        self.client.subscribe(topic,qos=0)
    
    def loop(self):
        self.client.loop_forever()



    

if __name__=="__main__":
    mqttclient=mqttFranka('127.0.0.1',1883,600)
    mqttclient.sub('franka_joint_motion')
    mqttclient.sub('franka_waypoints_motion')
    mqttclient.sub('franka_rfwaypoint_motion')
    mqttclient.sub('franka_dynamic_rel')
    mqttclient.sub('franka_grasp')
    mqttclient.loop()
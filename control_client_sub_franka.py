#!/usr/bin/python3.8
from os import path
import sys
from typing import ValuesView
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
        self.dynamix_rel=[0.15]
    
    def robot_init(self):
        #Connect to the robot
        self.robot =Robot(self.args.host)
        self.robot.set_default_behavior()
        self.robot.recover_from_errors()
        self.robot.set_dynamic_rel(0.15)
    
    def run(self):
        self.state=1
        fb_flag=[0,0]
        while len(self.taskqueque)!=0:
            #print(self.taskqueque)
            if self.taskqueque[0].topic=='franka_joint_motion':
                payloadstring=self.taskqueque[0].payload.decode()
                indexJ=payloadstring.find("jointmotion")
                indexRel=payloadstring.find("dynamix_rel")
                print(indexJ)
                print(indexRel)
                if indexJ==-1:
                    self.state=0
                    print("no jointmotion set value!")
                    self.taskqueque.pop(0)
                    return 
                if indexRel==-1:
                    self.dynamix_rel=[0.15]
                    print("cant find dynamix_rel,use the rel=0.15")
                
                # look up for joint pos value list 
                for i in range(indexJ,len(payloadstring)):
                    if payloadstring[i]=="[":
                        index=i
                        # some error for "["
                        if index>=indexRel and indexRel!=-1:
                            self.state=0
                            print("joint pos '[' is wrong!")
                            self.taskqueque.pop(0)
                            return 
                        
                        # mainstream continue
                        fb_flag[0]=i 
                    if payloadstring[i]=="]":
                        index=i
                        # some error for "]"
                        if index>=indexRel and indexRel!=-1:
                            self.state=0
                            print("joint pos ']' pos is wrong")
                            self.taskqueque.pop(0)
                            return 
                        fb_flag[1]=i
                        #some error for "]" "["
                        if fb_flag[0]>=fb_flag[1]:
                            self.state=0
                            print("joint pos '['  ']' pos wrong for each other！")
                            self.taskqueque.pop(0)
                            return 
                        break
                    #cant find "]" ,some wrong
                    if(i==indexRel) and (payloadstring[i]!="]"):
                        self.state=0
                        print("no ']' can be found!")
                        self.taskqueque.pop(0)
                        return 
                # if some wrong for joint pos string return 
                # if no wrong continue
                motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                motionString=''.join([i for i in motionString if not i.isalpha()])
                Value=list(map(float,motionString.split(','))) #get Value

                if indexRel!=-1:
                    # look up for joint velocity value list
                    for i in range(indexRel,len(payloadstring)):
                        if payloadstring[i]=="[":
                            index=i
                            #some error for "["
                            if index==len(payloadstring):
                                self.dynamix_rel=[0.15]
                                print("use default dynamix value!")
                                indexRel=-2
                            fb_flag[0]=i

                        if payloadstring[i]=="]":
                            index=i
                            #some error for "]"
                            fb_flag[1]=i
                            if fb_flag[0]>=fb_flag[1]:
                                self.dynamix_rel=[0.15]
                                print("set default dynamix_rel!")
                                indexRel=-2
                            break
                        if (i==len(payloadstring)-1) and (payloadstring[i]!="]"):
                            self.dynamix_rel=[0.15]
                            print("no default Value can be found,set default dynamix_rel!")
                            indexRel=-2
                            break
                    
                    if indexRel!=-2:
                        motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                        motionString=''.join([i for i in motionString if not i.isalpha()])
                        self.dynamix_rel=list(map(float,motionString.split(',')))
                    
                    if len(self.dynamix_rel)!=1:
                        print('dy_rel is wrong ,use the default value')
                        self.state=0
                        self.taskqueque.pop(0)
                        return 

                if len(Value)==7:
                    print("Value:::",Value)
                    print("dy_rel:::",self.dynamix_rel[0])
                    self.robot.set_dynamic_rel(self.dynamix_rel[0])
                    joint_motion=JointMotion(Value)
                    self.robot.move(joint_motion)
                    sleep(0.05)
                    self.taskqueque.pop(0)
                else:
                    self.state=0
                    print("joint value control input error!")
                    self.taskqueque.pop(0)
                    return 
            elif self.taskqueque[0].topic=="franka_waypoints_motion":
                # Waypoint(Affine(0.0, 0.0, -0.12), -0.2, Waypoint.Relative),
                # Waypoint(Affine(0.08, 0.0, 0.0), 0.0, Waypoint.Relative),
                # Waypoint(np.array([0.0,0.0,0.1]),np.array([0.0,0,3.14]),Waypoint.Relative),
                #[0.33,0,0.58],[0,3.14,-0.5]
                # Affine(double x, double y, double z, double a = 0.0, double b = 0.0, double c = 0.0)
                payloadstring=self.taskqueque[0].payload.decode()
                pathlist=payloadstring.split('|')
                pathmaplist=[]
                for i in range(len(pathlist)):
                    StringWP=pathlist[i]
                    indexWP=StringWP.find("waypoint")
                    indexDR=StringWP.find("dynamic_rel")
                    if (indexWP==-1) and (indexDR==-1):
                        self.state=0
                        print("cant find dynamic_rel and  waypoint set!")
                        self.taskqueque.pop(0)
                        return 
                    elif (indexDR!=-1) and (indexWP!=-1):
                        self.state=0
                        print("dynamic_rel and  waypoint at same time!")
                        self.taskqueque.pop(0)
                        return
                    elif (indexDR==-1) and (indexWP!=-1):
                        waypointMap={"waypoint":[0.33,0,0.58,0,0,0],
                                     "elbow":[0],
                                     "control_mode":"Absolute"}

                        cmdlist=StringWP.split('__')

                        if len(cmdlist)<=0:
                            self.state=0
                            print("no waypoint path set!")
                            self.taskqueque.pop(0)
                            return 
                        
                        for i in range(len(cmdlist)):
                            if i==0:                         
                                index=cmdlist[i].find('waypoint')
                                if index==-1:
                                    self.state=0
                                    print("1 waypoint path set is invalid!")
                                    self.taskqueque.pop(0)
                                    return 
                                else:
                                    index_L=cmdlist[i].find("[")
                                    index_R=cmdlist[i].find("]")
                                    print(cmdlist[i])
                                    if (index_L==-1) and (index_R==-1):
                                        self.state=0
                                        print("cant find '[' and ']'!")
                                        self.taskqueque.pop(0)
                                        return 
                                    elif (index_L==-1) and (index_R!=-1):
                                        self.state=0
                                        print("cant find '['!")
                                        self.taskqueque.pop(0)
                                        return 
                                    elif (index_L!=-1) and (index_R==-1):
                                        self.state=0
                                        print("cant find ']'!")
                                        self.taskqueque.pop(0)
                                        return
                                    else:
                                        if index_L>=index_R:
                                            self.state=0
                                            print("'[' and ']' set wrong!")
                                            self.taskqueque.pop(0)
                                            return
                                        else:
                                            print("index_L",index_L)
                                            print("index_R",index_R)
                                            motionString=cmdlist[i][index_L+1:index_R].replace(" ","")
                                            motionString=''.join([i for i in motionString if not i.isalpha()])
                                            Value=list(map(float,motionString.split(','))) # get Value
                                            if len(Value)!=6:
                                                self.state=0
                                                print("waypoint path length set is wrong!")
                                                self.taskqueque.pop(0)
                                                return
                                            else:
                                                waypointMap["waypoint"]=Value
                                pass
                            if i==1:
                                index=cmdlist[i].find('elbow')
                                if index==-1:
                                    waypointMap["elbow"]=[0]
                                    print("elbow Value use default Value!")
                                else:
                                    index_L=cmdlist[i].find("[")
                                    index_R=cmdlist[i].find("]")
                                    if (index_L==-1) and (index_R==-1):
                                        waypointMap["elbow"]=[0]
                                        print("elbow Value use default Value!")
                                    elif (index_L==-1) and (index_R!=-1):
                                        waypointMap["elbow"]=[0]
                                        print("elbow Value use default Value!")
                                    elif (index_L!=-1) and (index_R==-1):
                                        waypointMap["elbow"]=[0]
                                        print("elbow Value use default Value!")
                                    else:
                                        if index_L>=index_R:
                                            waypointMap["elbow"]=[0]
                                            print("elbow Value use default Value!")
                                        else:
                                            motionString=cmdlist[i][index_L+1:index_R].replace(" ","")
                                            motionString=''.join([i for i in motionString if not i.isalpha()])
                                            Value=list(map(float,motionString.split(','))) # get Value
                                            if len(Value)!=1:
                                                waypointMap["elbow"]=[0]
                                                print("elbow Value use default Value!")
                                            else:
                                                waypointMap["elbow"]=Value
                                pass

                            if i==2:
                                index=cmdlist[i].find('control_mode')
                                if index==-1:
                                    self.state=0
                                    print("1 waypoint path set is invalid!")
                                    self.taskqueque.pop(0)
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
                        index_dynamix_rel=StringWP.find("dynamic_rel")
                        if index_dynamix_rel==-1:
                            dynamic_rel_map["dynamic_rel"]=0.15
                        else:
                            index_L=StringWP.find("[")
                            index_R=StringWP.find("]")
                            if (index_L==-1) and (index_R==-1):
                                dynamic_rel_map["dynamic_rel"]=0.15
                                print("elbow Value use default Value!")
                            elif (index_L==-1) and (index_R!=-1):
                                dynamic_rel_map["dynamic_rel"]=0.15
                                print("elbow Value use default Value!")
                            elif (index_L!=-1) and (index_R==-1):
                                dynamic_rel_map["dynamic_rel"]=0.15
                                print("elbow Value use default Value!")
                            else:
                                if index_L>=index_R:
                                    dynamic_rel_map["dynamic_rel"]=0.15
                                    print("elbow Value use default Value!")
                                else:
                                    motionString=StringWP[index_L+1:index_R].replace(" ","")
                                    motionString=''.join([i for i in motionString if not i.isalpha()])
                                    Value=list(map(float,motionString.split(','))) # get Value
                                    if len(Value)!=1:
                                        dynamic_rel_map["dynamic_rel"]=0.15
                                        print("elbow Value use default Value!")
                                    else:
                                        dynamic_rel_map["dynamic_rel"]=Value[0]
                            pass
                        pathmaplist.append(dynamic_rel_map)
                #print(pathmaplist)
                waypointList=[]
                for j in range(len(pathmaplist)):
                    print(type(pathmaplist[j]))
                    print(pathmaplist[j].keys())
                   
                    if 'waypoint' in pathmaplist[j].keys():
                        wayp=pathmaplist[j]["waypoint"]
                        elbow_value=pathmaplist[j]["elbow"][0]
                        print(pathmaplist[j]["elbow"])
                        print(pathmaplist[j]["control_mode"])
                        affine=Affine(wayp[0],wayp[1],wayp[2],wayp[3],wayp[4],wayp[5])
                        if pathmaplist[j]["control_mode"]=="Relative":
                            waypoint=Waypoint(affine,elbow_value,Waypoint.Relative)
                        if pathmaplist[j]["control_mode"]=="Absolute":
                            waypoint=Waypoint(affine,elbow_value,Waypoint.Absolute)
                        waypointList.append(waypoint)
                    if 'dynamic_rel' in pathmaplist[j].keys():
                        print(pathmaplist[j]['dynamic_rel'])
                        self.robot.set_dynamic_rel(pathmaplist[j]['dynamic_rel'])  
                motion_down=WaypointMotion(waypointList)
                #self.robot.move(motion_down)  //some wrong for ruig
                self.taskqueque.pop(0)
                pass
            elif self.taskqueque[0].topic=="franka_rfwaypoint_motion":
                #Waypoint(np.array([0.3,0.0,0.58]),np.array([0.0,3.14,0.0]),Waypoint.Absolute),
                #Waypoint(np.array([0.4,0.0,0.58]),np.array([0.0,3.14,0.0]),Waypoint.Absolute),
                #Waypoint(np.array([0.3,0.0,0.58]),np.array([0.0,3.14,2.5]),Waypoint.Absolute),
                # Affine(double x, double y, double z, double a = 0.0, double b = 0.0, double c = 0.0)
                payloadstring=self.taskqueque[0].payload.decode()
                pathlist=payloadstring.split('|')
                pathmaplist=[]
                for i in range(len(pathlist)):
                    rfwaypointMap={"rfwaypoint_trans":[0.3,0.0,0.58],
                                   "rfwaypoint_r":[0.0,3.14,2.5],
                                   "control_mode":"Absolute"}
                    StringWP=pathlist[i]
                    indexT=StringWP.find("rfwaypoint_trans")
                    indexR=StringWP.find("rfwaypoint_R")
                    indexCM=StringWP.find("control_mode")


                    if(indexT==-1) or (indexR==-1):
                        self.state=0
                        print("cant find rfwaypoint_trans and rfwaypoint_R set!")
                        self.taskqueque.pop(0)
                        return
                    else:
                        if indexCM==-1:
                            rfwaypointMap["control_mode"]="Absolute"
                        else:
                            cmdlist=StringWP.split("__")
                            print(cmdlist)
                            for j in range(len(cmdlist)):
                                if j==0:
                                    indexTT=cmdlist[j].find("rfwaypoint_trans")
                                    if indexTT==-1:
                                        self.state=0
                                        print("cant find rfwaypoint_trans set!")
                                        self.taskqueque.pop(0)
                                        return
                                    else:
                                        index_L=cmdlist[j].find("[")
                                        index_R=cmdlist[j].find("]")
                                        if (index_L==-1) or (index_R==-1):
                                            self.state=0
                                            print("cant find '[' or ']' set!")
                                            self.taskqueque.pop(0)
                                            return
                                        else:
                                            motionString=cmdlist[j][index_L+1:index_R].replace(" ","")
                                            motionString=''.join([i for i in motionString if not i.isalpha()])
                                            Value=list(map(float,motionString.split(','))) # get Value
                                            if len(Value)!=3:
                                                self.state=0
                                                print("waypoint_trans path length set is wrong!")
                                                self.taskqueque.pop(0)
                                                return
                                            else:
                                                rfwaypointMap["rfwaypoint_trans"]=Value
                                    pass

                                if j==1:
                                    indexTT=cmdlist[j].find("rfwaypoint_R")
                                    if indexTT==-1:
                                        self.state=0
                                        print("cant find rfwaypoint_R set!")
                                        self.taskqueque.pop(0)
                                        return
                                    else:
                                        #print(cmdlist[j])
                                        index_L=cmdlist[j].find("[")
                                        index_R=cmdlist[j].find("]")
                                        if (index_L==-1) or (index_R==-1):
                                            self.state=0
                                            print("cant find '[' or ']' set!")
                                            self.taskqueque.pop(0)
                                            return
                                        else:
                                            motionString=cmdlist[j][index_L+1:index_R].replace(" ","")
                                            motionString=''.join([i for i in motionString if not i.isalpha()])
                                            #print(motionString)
                                            Value=list(map(float,motionString.split(','))) # get Value
                                            if len(Value)!=3:
                                                self.state=0
                                                print("waypoint_R path length set is wrong!")
                                                self.taskqueque.pop(0)
                                                return
                                            else:
                                                rfwaypointMap["rfwaypoint_R"]=Value
                                    pass

                                if j==2:
                                    index=cmdlist[j].find('control_mode')
                                    if index==-1:
                                        self.state=0
                                        print("1 waypoint path set is invalid!")
                                        self.taskqueque.pop(0)
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
                            
                                    pass
                    pathmaplist.append(rfwaypointMap)
                rfwaypointlist=[]
                for i in range(len(pathmaplist)):
                    trans=np.array(pathmaplist[i]["rfwaypoint_trans"])
                    rot=np.array(pathmaplist[i]["rfwaypoint_R"])
                    if pathmaplist[i]['control_mode']=="Absolute":
                        rfwaypoint=Waypoint(trans,rot,Waypoint.Absolute)
                    else:
                        rfwaypoint=Waypoint(trans,rot,Waypoint.Relative)
                    rfwaypointlist.append(rfwaypoint)
                print(rfwaypointlist)
                motion_down_rfwaypoint=MoveWaypointMotion(rfwaypointlist)
                #self.robot.move(motion_down_rfwaypoint)
                #self.robot.move(motion_down)  //some wrong for ruig
                self.taskqueque.pop(0)
                pass
            elif self.taskqueque[0].topic=="franka_dynamic_rel":
                self.robot.set_dynamic_rel(float(self.taskqueque[0].payload))
                self.taskqueque.pop(0)
                #self.robot.set_dynamic_rel()
            elif self.taskqueque[0].topic=="franka_grasp":
                graspString=self.taskqueque[0].payload.decode()
                graspsetlist=graspString.split("|")
                graspMap={"gripper_speed":0.02,"gripper_forced":20.0,"clamp":0}

                # today is over 
                
                for i in range(graspsetlist):
                    indexGS=graspsetlist[i].find("gripper_speed")
                    indexGF=graspsetlist[i].find("gripper_forced")
                    indexCL=graspsetlist[i].find("clamp")
                    if indexGS!=-1:
                        index_L=graspsetlist[i].find("[")
                        index_R=graspsetlist[i].find("]")
                        if (index_L==-1) or (index_R==-1):
                            self.state=0
                            print("cant find '[' or ']' set!")
                            self.taskqueque.pop(0)
                            return
                        else:
                            motionString=graspsetlist[i][index_L+1:index_R].replace(" ","")
                            motionString=''.join([i for i in motionString if not i.isalpha()])
                            Value=list(map(float,motionString.split(','))) # get Value
                            if len(Value)!=1:
                                self.state=0
                                graspMap["gripper_speed"]=0.02
                                print("gripper_speed path length set is wrong!use the default value")
                            else:
                                graspMap["gripper_speed"]=Value
                    if indexGF!=-1:
                        index_L=graspsetlist[i].find("[")
                        index_R=graspsetlist[i].find("]")
                        if (index_L==-1) or (index_R==-1):
                            self.state=0
                            print("cant find '[' or ']' set!")
                            self.taskqueque.pop(0)
                            return
                        else:
                            motionString=graspsetlist[i][index_L+1:index_R].replace(" ","")
                            motionString=''.join([i for i in motionString if not i.isalpha()])
                            Value=list(map(float,motionString.split(','))) # get Value
                            if len(Value)!=1:
                                self.state=0
                                graspMap["gripper_forced"]=20
                                print("gripper_forced path length set is wrong!use the default value")
                            else:
                                graspMap["gripper_forced"]=Value
                    if indexCL!=-1:
                        index_L=graspsetlist[i].find("[")
                        index_R=graspsetlist[i].find("]")
                        if (index_L==-1) or (index_R==-1):
                            self.state=0
                            print("cant find '[' or ']' set!")
                            self.taskqueque.pop(0)
                            return
                        else:
                            motionString=graspsetlist[i][index_L+1:index_R].replace(" ","")
                            motionString=''.join([i for i in motionString if not i.isalpha()])
                            Value=list(map(float,motionString.split(','))) # get Value
                            if len(Value)!=1:
                                self.state=0
                                print("clamp length set is wrong!break up!")
                                self.taskqueque.pop(0)
                                return
                            else:
                                graspMap["clamp"]=Value
                    else:
                        self.state=0
                        print("no clamp set!break up!")
                        self.taskqueque.pop(0)
                        return
                    gripper=self.robot.get_gripper()
                    gripper.gripper_speed=graspMap["gripper_speed"]
                    gripper.gripper_forced=graspMap["gripper_forced"]
                    if graspMap["clamp"]==1:
                        print(graspMap)
                        #gripper.clamp()
                    else:
                        print(graspMap)
                        #gripper.release(3.0)
                    
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
    mqttclient.loop()
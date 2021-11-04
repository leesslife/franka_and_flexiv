#!/usr/bin/python3.6
import paho.mqtt.client as mqtt
import sys
sys.path.append("/home/franka/home/rflexivx/")
from control_api import FlexivRobot
from time import sleep
import cv2
import numpy as np

'''
def on_connect(client,userdata,flasg,rc):
    print("Connected with result code:"+str(rc))

def on_message(client,userdata,msg):
    print(msg.topic+" "+str(msg.payload))


client=mqtt.Client()
client.on_connect=on_connect
client.on_message=on_message
client.connect("127.0.0.1",1883,1000)
client.subscribe('fifa',qos=0)
client.loop_forever()
'''
class RobotControl(object):
    def __init__(self):
        #Connect to the robot
        self.client=FlexivRobot("192.168.2.100","192.168.2.200")
        #Robot state
        #0 is stop
        #1 is runing
        self.state=0
        self.taskqueque=[]
    
    def read_joint_pos(self):
        return self.get_joint_pos()
    
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

    def op_flexiv_joint_motion(self):
        payloadstring=self.taskqueque[0].payload.decode()
        indexJ=payloadstring.find("jointmotion")   # try find jointmotion set value
        indexV=payloadstring.find("max_jnt_vel")   # try find max_velocity set value
        indexA=payloadstring.find("max_jnt_acc")   # try find max_acceleration set value

        joint_motionMap={"joint_motion":[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077],
                         "max_jnt_vel":[18,18,21,21,42,42,42],
                         "max_jnt_acc":[10.8,10.8,12.6,12.6,25.2,25.2,25.2]}
        if indexJ==-1:
            self.exit_error_info("no jointmotion set value!")              # if no jointmotion set
            return
        elif indexJ!=-1 and indexV==-1 and indexA==-1:
            joint_motionMap["max_jnt_vel"]=[6,6,7,7,14,14,14]
            joint_motionMap["max_jnt_acc"]=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
            jcmdlist=payloadstring.split('|')
            for i in range(len(jcmdlist)):
                indexJJ=jcmdlist[i].find("jointmotion")
                if indexJJ!=-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint pos Value is wrong!")
                        return 
                    joint_motionMap["joint_motion"]=Value
        elif indexJ!=-1 and indexV==-1 and indexA!=-1:
            joint_motionMap["max_jnt_vel"]=[6,6,7,7,14,14,14]
            jcmdlist=payloadstring.split('|')
            for i in range(len(jcmdlist)):
                indexJJ=jcmdlist[i].find("jointmotion")
                indexAA=jcmdlist[i].find("max_jnt_acc")
                if indexJJ!=-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint pos Value is wrong!")
                        return 
                    joint_motionMap["joint_motion"]=Value
                if indexAA!=-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint acceleration Value is wrong!")
                        return 
                    joint_motionMap["max_jnt_acc"]=Value
        elif indexJ!=-1 and indexV!=-1 and indexA==-1:
            joint_motionMap["max_jnt_acc"]=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
            jcmdlist=payloadstring.split('|')
            for i in range(len(jcmdlist)):
                indexJJ=jcmdlist[i].find("jointmotion")
                indexVV=jcmdlist[i].find("max_jnt_vel")
                if indexJJ!=-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint pos Value is wrong!")
                        return 
                    joint_motionMap["joint_motion"]=Value
                if indexVV!=-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint velocity Value is wrong!")
                        return 
                    joint_motionMap["max_jnt_vel"]=Value
        elif indexJ!=-1 and indexV!=-1 and indexA!=-1:
            jcmdlist=payloadstring.split('|')
            for i in range(len(jcmdlist)):
                indexAA=jcmdlist[i].find("max_jnt_acc")
                indexJJ=jcmdlist[i].find("jointmotion")
                indexVV=jcmdlist[i].find("max_jnt_vel")
                if indexJJ!=-1 and indexAA==-1 and indexVV==-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint pos Value is wrong!")
                        return 
                    print(Value)
                    joint_motionMap["joint_motion"]=Value
                elif indexVV!=-1 and indexJJ==-1 and indexAA==-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint velocity Value is wrong!")
                        return 
                    print(Value)
                    joint_motionMap["max_jnt_vel"]=Value
                elif indexAA!=-1 and indexVV==-1 and indexJJ==-1:
                    Value=self.get_motion_value(jcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint acceleration Value is wrong!")
                        return 
                    print(Value)
                    joint_motionMap["max_jnt_acc"]=Value
                else:
                    self.exit_error_info("cmd wrong!")
                    return 
        else:
            self.exit_error_info("some error state!")
            return
        print(joint_motionMap)
        self.client.move_jnt_pos(joint_motionMap["joint_motion"],max_jnt_vel=joint_motionMap["max_jnt_vel"],max_jnt_acc=joint_motionMap["max_jnt_acc"])
        sleep(0.05)
        self.taskqueque.pop(0)

    def op_flexiv_ptp_motion(self):
        payloadstring=self.taskqueque[0].payload.decode()
        indexP=payloadstring.find("ptpmotion")   # try find jointmotion set value
        indexV=payloadstring.find("max_jnt_vel")   # try find max_velocity set value
        indexA=payloadstring.find("max_jnt_acc")   # try find max_acceleration set value

        ptp_motionMap={"ptpmotion":[0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742],
                       "max_jnt_vel":[6,6,7,7,14,14,14],
                       "max_jnt_acc":[3.6,3.6,4.2,4.2,8.4,8.4,8.4]}
        if indexP==-1:
            self.exit_error_info("no ptp_motion set value!")              # if no jointmotion set
            return
        elif indexP!=-1 and indexV==-1 and indexA==-1:
            ptp_motionMap["max_jnt_vel"]=[6,6,7,7,14,14,14]
            ptp_motionMap["max_jnt_acc"]=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
            pcmdlist=payloadstring.split('|')
            for i in range(len(pcmdlist)):
                indexPP=pcmdlist[i].find("ptpmotion")
                if indexPP!=-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("carterian pos Value is wrong!")
                        return 
                    ptp_motionMap["ptpmotion"]=Value
        elif indexP!=-1 and indexV==-1 and indexA!=-1:
            ptp_motionMap["max_jnt_vel"]=[6,6,7,7,14,14,14]
            pcmdlist=payloadstring.split('|')
            for i in range(len(pcmdlist)):
                indexPP=pcmdlist[i].find("ptpmotion")
                indexAA=pcmdlist[i].find("max_jnt_acc")
                if indexPP!=-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("carterian pos Value is wrong!")
                        return 
                    ptp_motionMap["ptpmotion"]=Value
                if indexAA!=-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("carterian acceleration Value is wrong!")
                        return 
                    ptp_motionMap["max_jnt_acc"]=Value
        elif indexP!=-1 and indexV!=-1 and indexA==-1:
            ptp_motionMap["max_jnt_acc"]=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
            pcmdlist=payloadstring.split('|')
            for i in range(len(pcmdlist)):
                indexPP=pcmdlist[i].find("ptpmotion")
                indexVV=pcmdlist[i].find("max_jnt_vel")
                if indexPP!=-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("ptpmotion pos Value is wrong!")
                        return 
                    ptp_motionMap["ptpmotion"]=Value
                if indexVV!=-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("ptpmotion velocity Value is wrong!")
                        return 
                    ptp_motionMap["max_jnt_vel"]=Value
        elif indexP!=-1 and indexV!=-1 and indexA!=-1:
            pcmdlist=payloadstring.split('|')
            for i in range(len(pcmdlist)):
                indexAA=pcmdlist[i].find("max_jnt_acc")
                indexJJ=pcmdlist[i].find("ptpmotion")
                indexVV=pcmdlist[i].find("max_jnt_vel")
                if indexJJ!=-1 and indexAA==-1 and indexVV==-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint pos Value is wrong!")
                        return 
                    print(Value)
                    ptp_motionMap["ptpmotion"]=Value
                elif indexVV!=-1 and indexJJ==-1 and indexAA==-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("joint velocity Value is wrong!")
                        return 
                    print(Value)
                    ptp_motionMap["max_jnt_vel"]=Value
                elif indexAA!=-1 and indexVV==-1 and indexJJ==-1:
                    Value=self.get_motion_value(pcmdlist[i],7)
                    if Value==-9999:
                        self.exit_error_info("ptpmotion acceleration Value is wrong!")
                        return 
                    print(Value)
                    ptp_motionMap["max_jnt_acc"]=Value
                else:
                    self.exit_error_info("cmd wrong!")
                    return 
        else:
            self.exit_error_info("some error state!")
            return
        print(ptp_motionMap)
        PValue=np.array(ptp_motionMap["ptpmotion"])
        max_jnt_vel=np.array(ptp_motionMap["max_jnt_vel"])
        max_jnt_acc=np.array(ptp_motionMap["max_jnt_acc"])
        self.client.move_ptp(PValue,max_jnt_vel=max_jnt_vel,max_jnt_acc=max_jnt_acc)
        sleep(0.05)
        self.taskqueque.pop(0)
           
    def run(self):
        self.state=1
        while len(self.taskqueque)!=0:
            #print(self.taskqueque)
            if self.taskqueque[0].topic=="flexiv_joint_motion":
                self.op_flexiv_joint_motion()
            elif self.taskqueque[0].topic=="flexiv_ptp_motion":
                self.op_flexiv_ptp_motion()          
            else:
                self.exit_error_info("error control flag!")
                return
        self.state=0

flexiv_robot=RobotControl()

def on_connect(client,userdata,flasg,rc):
    print("Connected with result code:"+str(rc))

def on_message(client,userdata,msg):
    if flexiv_robot.state==0:
        flexiv_robot.taskqueque.append(msg)
        flexiv_robot.run()
    if flexiv_robot.state==1:
        flexiv_robot.taskqueque.append(msg)

class mqttFlexiv(object):
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
    mqttclient=mqttFlexiv("127.0.0.1",1883,600)
    mqttclient.sub('flexiv_joint_motion')
    mqttclient.sub('flexiv_ptp_motion')
    mqttclient.loop()


   
    




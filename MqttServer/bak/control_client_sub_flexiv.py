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

    def run(self):
        self.state=1
        fb_flag=[0,0]
        max_jnt_vel=[6,6,7,7,14,14,14]           
        max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
        index=0
        while len(self.taskqueque)!=0:
            #print(self.taskqueque)
            if self.taskqueque[0].topic=="flexiv_joint_motion":
                payloadstring=self.taskqueque[0].payload.decode()
                indexJ=payloadstring.find("jointmotion")   # try find jointmotion set value
                indexV=payloadstring.find("max_jnt_vel")   # try find max_velocity set value
                indexA=payloadstring.find("max_jnt_acc")   # try find max_acceleration set value

                if indexJ==-1:                  # if no jointmotion set
                    self.state=0
                    print("no jointmotion set value!")
                    self.taskqueque.pop(0)
                    return
                # if no max_velocity set 
                if indexV==-1:
                    max_jnt_vel=[6,6,7,7,14,14,14]  # use default value
                    print("set default mat_velocity value")
                # if no max_acceleration set 
                if indexA==-1:
                    max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4] #use default value
                    print("set default max_acceleration value")

                # look up for joint pos value list
                for i in range(indexJ,len(payloadstring)):
                    if payloadstring[i]=="[":
                        index=i
                        # some error for "[" 
                        if index>=indexV and indexV!=-1:
                            self.state=0
                            print("joint pos '[' pos is wrong!")
                            self.taskqueque.pop(0)
                            return
                        # mainstream continue
                        fb_flag[0]=i
                    if payloadstring[i]=="]":
                        index=i
                        # some error for "]" 
                        if index>=indexV and indexV!=-1:
                            self.state=0
                            print("joint pos ']' pos is wrong!")
                            self.taskqueque.pop(0)
                            return
                        # mainstream continue
                        fb_flag[1]=i
                        # some error for "]" "["
                        if fb_flag[0]>=fb_flag[1]:
                            self.state=0
                            print("joint pos '['  ']' pos wrong for each other！")
                            self.taskqueque.pop(0)
                            return
                        break
                    # cant find "]",some wrong
                    if (i==indexV) and (payloadstring[i]!="]"):
                        self.state=0
                        print("no ']' can be found！")
                        self.taskqueque.pop(0)
                        return
                # if some wrong for joint pos string return 
                # if no wrong continue
                motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                motionString= ''.join([i for i in motionString if not i.isalpha()])
                Value=list(map(float,motionString.split(',')))  # get Value 
                if indexV!=-1:
                    # look up for joint velocity value list
                    for i in range(indexV,len(payloadstring)):
                        if payloadstring[i]=="[":
                            index=i
                            # some error for "["
                            if index>=indexA:
                                max_jnt_vel=[6,6,7,7,14,14,14]   #use default max_velocity
                                print("use default max_velocity value!")
                                indexV=-2
                            fb_flag[0]=i   #continue
                        if payloadstring[i]=="]":
                            index=i
                            # some error for "]"
                            if index>=indexA:
                                max_jnt_vel=[6,6,7,7,14,14,14]   #use default max_velocity
                                print("set default mat_velocity value")
                                indexV=-2
                            # continue
                            fb_flag[1]=i
                            # some error for "]" "["
                            if fb_flag[0]>=fb_flag[1]:
                                max_jnt_vel=[6,6,7,7,14,14,14]
                                print("set default mat_velocity value")
                                indexV=-2
                            break
                        # cant find "]",some wrong
                        if (i==indexA) and (payloadstring[i]!="]"):
                            max_jnt_vel=[6,6,7,7,14,14,14]
                            print("no velocity can be found ,set default max_velocity value")
                            indexV=-2
                            break
                    # if no wrong for max _velocity ,we use the set value list
                    if indexV!=-2:
                        motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                        motionString= ''.join([i for i in motionString if not i.isalpha()])
                        max_jnt_vel=list(map(float,motionString.split(',')))

                    #print(max_jnt_vel)
                    if len(max_jnt_vel)!=7:
                        print("max_jnt_vel len is wrong!")
                        self.state=0
                        self.taskqueque.pop(0)
                        return

                if indexA!=-1:
                    # look up for joint acceleration value list
                    for i in range(indexA,len(payloadstring)):
                        if payloadstring[i]=="[":
                            index=i
                            # if "[" at the last pos ,some wrong
                            if index==len(payloadstring)-1:
                                max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4] #use default max_velocity
                                print("set default mat_acceleration value")
                                indexA=-2
                            fb_flag[0]=i

                        if payloadstring[i]=="]":
                            index=i
                            fb_flag[1]=i
                            #  # some error for "]" "["
                            if fb_flag[0]>=fb_flag[1]:
                                max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
                                print("set default mat_acceleration value")
                                indexA=-2
                            break
                        # cant find "]",some wrong
                        if (i==len(payloadstring)-1) and (payloadstring[i]!="]"):
                            max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
                            print("no acceleration can be found ,set default mat_acceleration value")
                            indexA=-2
                            break

                    if indexA!=-2:
                        motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                        motionString= ''.join([i for i in motionString if not i.isalpha()])
                        max_jnt_acc=list(map(float,motionString.split(',')))

                    if len(max_jnt_acc)!=7:
                        print("max_jnt_acc len is wrong!")
                        self.state=0
                        self.taskqueque.pop(0)
                        return

                if len(Value)==7:
                    self.client.move_jnt_pos(Value,
                                             max_jnt_vel=max_jnt_vel,
                                             max_jnt_acc=max_jnt_acc)
                    print(Value)
                    print(max_jnt_vel)
                    print(max_jnt_acc)
                    sleep(0.05)
                    self.taskqueque.pop(0)
                else:
                    self.state=0
                    print("joint value control input error!")
                    self.taskqueque.pop(0)
                    return


            elif self.taskqueque[0].topic=="flexiv_ptp_motion":
            
                payloadstring=self.taskqueque[0].payload.decode()
                indexC=payloadstring.find("ptpmotion")     # try find Cartesian set value
                indexV=payloadstring.find("max_jnt_vel")   # try find max_velocity set value
                indexA=payloadstring.find("max_jnt_acc")   # try find max_acceleration set value

                if indexC==-1:                  # if no Cartesian set
                    self.state=0
                    print("no set the Cartesian value!")
                    self.taskqueque.pop(0)
                    return
                # if no max_velocity set 
                if indexV==-1:
                    max_jnt_vel=[6,6,7,7,14,14,14]  # use default value
                    print("set default mat_velocity value")
                # if no max_acceleration set 
                if indexA==-1:
                    max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4] #use default value
                    print("set default max_acceleration value")

                # look up for cartesian value list
                for i in range(indexC,len(payloadstring)):
                    if payloadstring[i]=="[":
                        index=i
                        # some error for "[" 
                        if index>=indexV and indexV!=-1:
                            self.state=0
                            print("tcp pos '[' pos is wrong!")
                            self.taskqueque.pop(0)
                            return
                        # mainstream continue
                        fb_flag[0]=i
                    if payloadstring[i]=="]":
                        index=i
                        # some error for "]" 
                        if index>=indexV and indexV!=-1:
                            self.state=0
                            print("tcp pos ']' pos is wrong!")
                            self.taskqueque.pop(0)
                            return
                        # mainstream continue
                        fb_flag[1]=i
                        # some error for "]" "["
                        if fb_flag[0]>=fb_flag[1]:
                            self.state=0
                            print("tcp pos '['  ']' pos wrong for each other！")
                            self.taskqueque.pop(0)
                            return
                        break
                    # cant find "]",some wrong
                    if (i==indexV) and (payloadstring[i]!="]"):
                        self.state=0
                        print("no ']' can be found！")
                        self.taskqueque.pop(0)
                        return
                # if some wrong for joint pos string return 
                # if no wrong continue
                motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                motionString= ''.join([i for i in motionString if not i.isalpha()])
                Value=list(map(float,motionString.split(',')))  # get Value 
                if indexV!=-1:
                    # look up for joint velocity value list
                    for i in range(indexV,len(payloadstring)):
                        if payloadstring[i]=="[":
                            index=i
                            # some error for "["
                            if index>=indexA:
                                max_jnt_vel=[6,6,7,7,14,14,14]   #use default max_velocity
                                print("use default max_velocity value!")
                                indexV=-2
                            fb_flag[0]=i   #continue
                        if payloadstring[i]=="]":
                            index=i
                            # some error for "]"
                            if index>=indexA:
                                max_jnt_vel=[6,6,7,7,14,14,14]   #use default max_velocity
                                print("set default mat_velocity value!")
                                indexV=-2
                            # continue
                            fb_flag[1]=i
                            # some error for "]" "["
                            if fb_flag[0]>=fb_flag[1]:
                                max_jnt_vel=[6,6,7,7,14,14,14]
                                print("set default mat_velocity value")
                                indexV=-2
                            break
                        # cant find "]",some wrong
                        if (i==indexA) and (payloadstring[i]!="]"):
                            max_jnt_vel=[6,6,7,7,14,14,14]
                            print("no velocity can be found ,set default max_velocity value")
                            indexV=-2
                            break
                    # if no wrong for max _velocity ,we use the set value list
                    if indexV!=-2:
                        motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                        motionString= ''.join([i for i in motionString if not i.isalpha()])
                        max_jnt_vel=list(map(float,motionString.split(',')))

                    #print(max_jnt_vel)
                    if len(max_jnt_vel)!=7:
                        print("max_jnt_vel len is wrong!")
                        self.state=0
                        self.taskqueque.pop(0)
                        return

                if indexA!=-1:
                    # look up for joint acceleration value list
                    for i in range(indexA,len(payloadstring)):
                        if payloadstring[i]=="[":
                            index=i
                            # if "[" at the last pos ,some wrong
                            if index==len(payloadstring)-1:
                                max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4] #use default max_velocity
                                print("set default mat_acceleration value")
                                indexA=-2
                            fb_flag[0]=i

                        if payloadstring[i]=="]":
                            index=i
                            fb_flag[1]=i
                            #  # some error for "]" "["
                            if fb_flag[0]>=fb_flag[1]:
                                max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
                                print("set default mat_acceleration value")
                                indexA=-2
                            break
                        # cant find "]",some wrong
                        if (i==len(payloadstring)-1) and (payloadstring[i]!="]"):
                            max_jnt_acc=[3.6,3.6,4.2,4.2,8.4,8.4,8.4]
                            print("no acceleration can be found ,set default mat_acceleration value")
                            indexA=-2
                            break

                    if indexA!=-2:
                        motionString=payloadstring[fb_flag[0]+1:fb_flag[1]].replace(" ","")
                        motionString= ''.join([i for i in motionString if not i.isalpha()])
                        max_jnt_acc=list(map(float,motionString.split(',')))

                    if len(max_jnt_acc)!=7:
                        print("max_jnt_acc len is wrong!")
                        self.state=0
                        self.taskqueque.pop(0)
                        return

                if len(Value)==7:
                    Value=np.array(Value)
                    max_jnt_vel=np.array(max_jnt_vel)
                    max_jnt_acc=np.array(max_jnt_acc)
                    self.client.move_ptp(Value,
                                         max_jnt_vel=max_jnt_vel,
                                         max_jnt_acc=max_jnt_acc)
                    print(Value)
                    print(max_jnt_vel)
                    print(max_jnt_acc)
                    sleep(0.05)
                    self.taskqueque.pop(0)
                else:
                    self.state=0
                    print("joint value control input error!")
                    self.taskqueque.pop(0)
                    return
            
            
            else:
                self.state=0
                print("error control flag!")
                self.taskqueque.pop(0)
                return
        self.state=0

flexiv_robot=RobotControl()

def on_connect(client,userdata,flasg,rc):
    print("Connected with result code:"+str(rc))

def on_message(client,userdata,msg):
    if flexiv_robot.state==0:
        print(flexiv_robot.client.is_moving())
        flexiv_robot.taskqueque.append(msg)
        flexiv_robot.run()
    if flexiv_robot.state==1:
        print(111111111)
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


   
    




#!/usr/bin/python3.8
from ctypes import py_object
from time import sleep
import paho.mqtt.client as mqtt

client=mqtt.Client()
client.connect('127.0.0.1',1883,100)
client.publish("franka_dynamic_rel",payload=0.15,qos=0)

#client.publish("franka_grasp",payload="gripper_speed=[0.02]__gripper_forced=[20.0]__clamp=[0]",qos=0)
client.publish('franka_joint_motion',payload='jointmotion=[-0.1030304962792744,-0.35080076630911905,0.02602974068151231,-2.2289171381306514,0.07777110726569612,2.0893894506825093,0.7032888972014189]\
                                              dynamix_rel=[0.15]',qos=0)

client.publish('franka_joint_motion',payload='jointmotion=[-0.06827773589518435,0.7349156716162698,0.042826138296654596,-1.810947657966147,0.07783103891141628,2.537591380689967,0.6111039300014576]\
                                              dynamix_rel=[0.15]',qos=0)

client.publish("franka_grasp",payload="gripper_speed=[0.02]__gripper_forced=[20.0]__clamp=[1]",qos=0)

client.publish('franka_joint_motion',payload='jointmotion=[0.07292698271755586,-0.14292802196009116,-0.11688850735781484,-2.531209969035366,-0.03786461907625198,3.7413173255655496,2.368267249240555]\
                                              dynamix_rel=[0.15]',qos=0)

#client.publish('franka_joint_motion',payload='jointmotion=[-0.06827773589518435,0.7349156716162698,0.042826138296654596,-1.810947657966147,0.07783103891141628,2.537591380689967,0.6111039300014576]\
#                                              dynamix_rel=[0.15]',qos=0)

#client.publish("franka_grasp",payload="gripper_speed=[0.02]__gripper_forced=[20.0]__clamp=[0]",qos=0)
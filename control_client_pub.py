#!/usr/bin/python3.8
from ctypes import py_object
from time import sleep
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

# connect mqttserver 
client=mqtt.Client()
client.on_connect=on_connect
client.on_message=on_message
client.connect('127.0.0.1',1883,100)
#client.publish("franka_dynamic_rel",payload=0.15,qos=0)

#client.publish('franka_rfwaypoint_motion',payload="rfwaypoint_trans=[0.3,0.0,0.58]__rfwaypoint_R=[0.0,3.14,0.0]__control_mode=Absolute|\
#                                                   rfwaypoint_trans=[0.4,0.0, 0.48]__rfwaypoint_R=[0.0,3.14,0.0]__control_mode=Absolute|\
#                                                   rfwaypoint_trans=[0.3,0.0,0.58]__rfwaypoint_R=[0.0,3.14,2.5]__control_mode=Absolute|\
#                                                   rfwaypoint_trans=[0.0,-0.1,0.0]__rfwaypoint_R=[0.0,3.14,2.5]__control_mode=Relative",qos=0)

#gripper.gripper_speed=0.02
#gripper.gripper_forced=20.0

#client.publish("franka_grasp",payload="gripper_speed=[0.02]__gripper_forced=[20.0]__clamp=[1]",qos=0)
# connect franka server
# static constexpr double max_translation_velocity {1.7}; // [m/s]
# static constexpr double max_rotation_velocity {2.5}; // [rad/s]
# static constexpr double max_elbow_velocity {2.175}; // [rad/s]
# static constexpr double max_translation_acceleration {13.0}; // [m/s²]
# static constexpr double max_rotation_acceleration {25.0}; // [rad/s²]
# static constexpr double max_elbow_acceleration {10.0}; // [rad/s²]
# static constexpr double max_translation_jerk {6500.0}; // [m/s³]
# static constexpr double max_rotation_jerk {12500.0}; // [rad/s³]
# static constexpr double max_elbow_jerk {5000.0}; // [rad/s³]
# robot.set_dynamic_rel(0.15) this factor times max_value 1.7x0.15
# default value is 0.15 1.7*0.15
# client.publish('franka_joint_motion',payload='jointmotion=[1, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]|\
#                                              dynamic_rel=[0.15]',qos=0)

#client.publish("franka_grasp",payload="gripper_speed=[0.02]__gripper_forced=[20.0]__clamp=[0]",qos=0)
#client.publish('franka_joint_motion',payload='jointmotion=[0.1, -0.385398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]\
#                                              dynamix_rel=[0.15]',qos=0)

# waypoint=[x,y,z,a,b,c]  eular=[a,b,c] 
# elbow default=0
# control_model default=Absolute
# some wrong for ruig
#client.publish('franka_waypoints_motion',payload='waypoint=[0.33, 0,   0.58, 0, 0, 0]__elbow=[-0.2]__control_mode=Absolute|\
#                                                  waypoint=[0.33, 0.2, 0.58, 0, 0, 0]__elbow=[0]|\
#                                                  waypoint=[0.33, 0,   0.58, 0, 0, 0]|\
#                                                  dynamic_rel=[0.05]',qos=0)                                        
#client.publish('franka_joint_motion',payload='0.1, -0.785398163397448279, 0.1, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279',qos=0)
#client.publish('franka_1oint_motion',payload='0.1, -0.785398163397448279, 0.2, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279',qos=0)
#client.publish('franka_waypoint_motion',payload='0.1, -0.785398163397448279, 0.3, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279',qos=0)

# flexiv control api for joint value and tcp value 
#client.publish('flexiv_joint_motion',payload="jointmotion=[-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077]|\
#                                              max_jnt_vel=[18,18,21,21,42,42,42]|\
#                                              max_jnt_acc=[10.8,10.8,12.6,12.6,25.2,25.2,25.2]",qos=0)

#client.publish('flexiv_joint_motion',payload="jointmotion= [-0.0155,-0.451,0.619,1.62,0.001596,0.5288,0.004077]| \
#                                              max_jnt_vel=[18,18,21,21,42,42,42]| \
#                                              max_jnt_acc=[10.8,10.8,12.6,12.6,25.2,25.2,25.2]",qos=0)

#client.publish('flexiv_joint_motion',payload="jointmotion= [-0.0155,-0.451,1.619,1.62,0.001596,0.5288,0.004077]",qos=0)

#
client.publish('flexiv_ptp_motion',payload="ptpmotion= [0.6518,0.211,0.607,0.1675,-0.237,0.94,0.1742]| \
                                            max_jnt_vel=[18,18,21,21,42,42,42]| \
                                            max_jnt_acc=[10.8,10.8,12.6,12.6,25.2,25.2,25.2]",qos=0)

client.publish('flexiv_ptp_motion',payload="ptpmotion= [ 0.5518,0.211,0.607,0.1675,-0.237,0.94,0.1742]",qos=0)
#
client.publish('flexiv_ptp_motion',payload="ptpmotion=  [0.4518,0.411,0.307,0.1675,-0.237,0.94,0.1742]| \
                                            max_jnt_vel=[18,18,21,21,42,42,42]| \
                                            max_jnt_acc=[10.8,10.8,12.6,12.6,25.2,25.2,25.2]",qos=0)

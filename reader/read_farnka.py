#!/usr/bin/python3.8
from time import sleep
import paho.mqtt.client as mqtt

# connect mqttserver 
client=mqtt.Client()
client.connect('127.0.0.1',1883,100)
client.publish('read_franka_j',payload="[-1,0.5,jlog1]",qos=0)
#client.publish('read_flexiv_joint',payload="[-1,  0.5,log1]",qos=0)

client.publish('read_franka_c',payload="[-1,0.5,clog1]",qos=0)


#!/usr/bin/python3.8
from ctypes import py_object
from time import sleep
import paho.mqtt.client as mqtt



client=mqtt.Client()
client.connect('127.0.0.1',1883,100)
client.publish("franka_dynamic_rel",payload=0.15,qos=0)

client.publish('read_franka_joint',payload="[1, 0.5,franka_J_save1]",qos=0)
client.publish('read_franka_cartesian',payload="[1, 0.5,franka_P_save1]",qos=0)
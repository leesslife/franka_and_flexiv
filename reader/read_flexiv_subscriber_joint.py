#!/usr/bin/python3.6
from time import sleep
import paho.mqtt.client as mqtt


def on_connect(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

client=mqtt.Client()
client.on_connect=on_connect
client.on_message=on_message
client.connect('127.0.0.1',1883,100)
client.publish("'read_flexiv_jv'",payload=0.15,qos=0)
client.loop_forever()
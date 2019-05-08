---
layout: post
title: Setup MQTT mosquitto on RaspberryPI
categories: MQTT
tags: [mqtt, mosquitto, python, raspberryPi]
public: true

---
![](/images/mosquitto_rpi_logo.jpg)

MQTT is a lightweight publish/subscribe messaging protocol.  
Eclipse Mosquitto is an open source message broker that implements the MQTT.

MQTT stands for MQ Telemetry Transport, MQTT is a tcp based subscribe and publish messaging protocol.  
MQTT is main protocol for IOT.  


# MQTT terms 
- MQTT Broker: MQTT server
- Topic: Name/String define specific channel
- Publisher: Send/Pub messages
- Subscriber: Receive me
- Message
- MQTT Client: `subscribe` or `publish` to specific `topic`


![](/images/2019-05-08-14-27-04.png)

# Lab
- PI 3+
- Raspbian Stretch
- > Install tmux for ssh multi screen
- Python paho-mqtt (mqtt version 3.1/3.1.1)


# RaspberryPI Install 

```bash
#server
sudo apt-get install mosquitto
#client
sudo apt-get install mosquitto-clients
```

## Test install
- Run tmux `tmux`
- Pane 1 (subscriber)
```bash
#-t topic name
mosquitto_sub -t "test"
```

- Pane 2 (publisher)
```bash
mosquitto_pub -m "message from mosquitto_pub client" -t "test"
```

![](/images/2019-05-08-18-40-55.png)

# Python
```bash
pip install paho-mqtt
```

## Subscriber
```python
import paho.mqtt.client as mqtt #import the client1
import time
TOPIC="house/bulbs/bulb1"

def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

broker_address="192.168.2.100"
client = mqtt.Client() #create new instance
client.on_message=on_message #attach function to callback
client.connect(broker_address) #connect to broker
client.loop_start() #start the loop
client.subscribe(TOPIC)
time.sleep(20) # wait
client.loop_stop() #stop the loop
```

## Publisher
```python
import paho.mqtt.client as mqtt #import the client1
broker_address="192.168.2.100"
TOPIC="house/bulbs/bulb1"

def on_log(client, userdata, level, buf):
    print("log: ",buf)

client = mqtt.Client() #create new instance
client.on_log = on_log
client.connect(broker_address) #connect to broker
client.publish(TOPIC,"OFF")
```
## Reference
- [How to Install Mosquitto MQTT Broker/Server on Ubuntu](https://www.vultr.com/docs/how-to-install-mosquitto-mqtt-broker-server-on-ubuntu-16-04)
- [Beginners Guide To The Paho MQTT Python Client](http://www.steves-internet-guide.com/into-mqtt-python-client/)
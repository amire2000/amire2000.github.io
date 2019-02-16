---
layout: post
title: MQTT mosquitto hello world
categories: MQTT
tags: [mqtt, mosquitto, python]
---

# Ubuntu install 
- Server
```bash
sudo apt-get install mosquitto
```
- Client
```bash
sudo apt-get install mosquitto-clients
```

## Test install
- Terminal 1
```bash
#-t topic name
mosquitto_sub -t "test"
```

- Terminal 2
```bash
mosquitto_pub -m "message from mosquitto_pub client" -t "test"
```


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
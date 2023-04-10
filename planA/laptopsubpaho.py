import paho.mqtt.client as mqtt
import time

MQTT_BROKER = '192.168.1.102'
#remember to change this to your laptop IP Address

def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    client.subscribe("TableNo")

def on_message(client, userdata, message):
    print("received message: " ,str(message.payload.decode("utf-8")))


client = mqtt.Client("Turtlebot")
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("roger", "password")
client.connect(MQTT_BROKER, 1883)
client.loop_forever()
import time
import ubinascii
import machine
from umqtt.simple import MQTTClient

MQTT_BROKER = '192.168.1.102'
CLIENT_ID = ubinascii.hexlify(machine.unique_id())
user = "roger"
password = "password"
MQTT_TOPIC = "TableNo"

def reset():
    print("Resetting...")
    time.sleep(5)
    machine.reset()

    
def main():    
    print(f"client {CLIENT_ID} to mqtt broker: {MQTT_BROKER}\n")
    mqttClient = MQTTClient(CLIENT_ID, server=MQTT_BROKER, user=user, password=password, keepalive=60)
    mqttClient.connect()
    
    while True:
        Table = input("Enter table number: ")
        print(f"Publishing Table number :: {Table}")
        mqttClient.publish(MQTT_TOPIC, str(Table).encode())
        time.sleep(3)
    mqttClient.disconnect()
    
if __name__ == "__main__":
    try:
        main()
    except OSError as e:
        print("Error: " + str(e))
        reset()
    



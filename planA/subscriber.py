import time
import ubinascii
import machine
from umqtt.simple import MQTTClient

MQTT_BROKER = '192.168.128.219'
#remember to change this to your laptop IP Address
CLIENT_ID = ubinascii.hexlify(machine.unique_id())
user = "roger"
#remember to change this to user in your passwordfile
password = "password"
#remember to change this to password in your passwordfile
MQTT_TOPIC = b"user"

#Set IO Pin config
sg90 = PWM(Pin(32, mode=Pin.OUT))
sg90.freq(50)

# 0.5ms/20ms = 0.025 = 2.5% duty cycle
# 2.4ms/20ms = 0.12 = 12% duty cycle

# 0.025*1024=25.6
# 0.12*1024=122.88

def sub_cb(topic, msg):
    print((topic, msg))
    while True:
        sg90.duty(15)
        time.sleep(1)
        sg90.duty(30)
        time.sleep(1)
        #does it return back to position?

def reset():
    print("Resetting...")
    time.sleep(5)
    machine.reset()


def main():
    mqttClient = MQTTClient(CLIENT_ID, server=MQTT_BROKER, user=user, password=password, keepalive=60)
    mqttClient.set_callback(sub_cb)
    mqttClient.connect()
    mqttClient.subscribe(MQTT_TOPIC)
    print(f"Connected to MQTT  Broker :: {MQTT_BROKER}, and waiting for callback function to be called!")
    while True:
        if False:
            # Blocking wait for message
            mqttClient.wait_msg()
        else:
            # Non-blocking wait for message
            mqttClient.check_msg()
            # Then need to sleep to avoid 100% CPU usage (in a real
            # app other useful actions would be performed instead)
            global last_ping
            if (time.time() - last_ping) >= ping_interval:
                mqttClient.ping()
                last_ping = time.time()
                now = time.localtime()
                print(f"Pinging MQTT Broker, last ping :: {now[0]}/{now[1]}/{now[2]} {now[3]}:{now[4]}:{now[5]}")
            time.sleep(1)
            
    print("Disconnecting...")
    mqttClient.disconnect()
    
if __name__ == "__main__":
    try:
        main()
    except OSError as e:
        print("Error: " + str(e))
        reset()

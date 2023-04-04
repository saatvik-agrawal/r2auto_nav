import time
import ubinascii
import machine
from umqtt.simple import MQTTClient

#MQTT topic is called "TableNo"

MQTT_BROKER = '192.168.128.219'
#remember to change this to your laptop IP Address
CLIENT_ID = ubinascii.hexlify(machine.unique_id())
user = "roger"
#remember to change this to user in your passwordfile
password = "password"
#remember to change this to password in your passwordfile
MQTT_TOPIC = "TableNo"

#servo setup
sg90 = PWM(Pin(35, mode=Pin.OUT))
sg90.freq(50)
sg90.duty(15)

#Button and Display setup
push_button1 = Pin(23, Pin.IN, Pin.PULL_UP) # input as table 1
push_button2 = Pin(22, Pin.IN, Pin.PULL_UP)  # input as table 2
push_button3 = Pin(21, Pin.IN, Pin.PULL_UP)  # input as table 3
push_button4 = Pin(19, Pin.IN, Pin.PULL_UP)  # input as table 4
push_button5 = Pin(18, Pin.IN, Pin.PULL_UP)  # input as table 5
push_button6 = Pin(5, Pin.IN, Pin.PULL_UP)  # input as table 6
push_button7 = Pin(4, Pin.IN, Pin.PULL_UP)  # input as cancelOrder

current_table = -1 #-1 means no table assigned or cancelled

led_A = Pin(13, Pin.OUT)    # number in is Output
led_B = Pin(2, Pin.OUT)    # number in is Output
led_C = Pin(14, Pin.OUT)    # number in is Output
led_D = Pin(27, Pin.OUT)    # number in is Output
led_E = Pin(26, Pin.OUT)    # number in is Output
led_F = Pin(25, Pin.OUT)    # number in is Output
led_G = Pin(33, Pin.OUT)    # number in is Output
led_DP = Pin(32, Pin.OUT)    # number in is Output

segments =  [led_A,led_B,led_C,led_D,led_E,led_F,led_G]

num = {' ':[0,0,0,0,0,0,0],
    '1':[0,0,0,0,1,1,0],
    '2':[1,0,0,1,1,0,1],
    '3':[1,0,0,1,1,1,1],
    '4':[0,0,1,0,1,1,1],
    '5':[1,0,1,1,0,1,1],
    '6':[1,1,1,1,0,1,1],
    'C':[1,0,1,1,0,0,0]}

#Ultrasonic sensor setup
trigger_pin = Pin(15, Pin.OUT)
echo_pin = Pin(12, Pin.IN)
       

def reset():
    print("Resetting...")
    time.sleep(5)
    machine.reset()


def display(n):
    print("in display")
    for loop in range(0,7):
        segments[loop].value(num[n][loop])
        print(loop)
        time.sleep(0.5)
    time.sleep(3)
    print(n)

def buttons():
    if not push_button1.value():
        display('1')
        table = 1
    elif not push_button2.value():
        display('2')
        table = 2
    elif not push_button3.value():
        display('3')
        table = 3
    elif not push_button4.value():
        display('4')
        table = 4
    elif not push_button5.value():
        display('5')
        table = 5
    elif not push_button6.value():
        display('6')
        table = 6
    elif not push_button7.value():
        display('C')
        table = -1
        
def ultrasonic_distance(): #read distance for servo, locally done
    trigger_pin.on()
    time.sleep_us(10)
    trigger_pin.off()

    pulse_duration = time_pulse_us(echo_pin, 1)
    distance = pulse_duration / 58.0

    return distance

def main():    
    print(f"client {CLIENT_ID} to mqtt broker: {MQTT_BROKER}\n")
    mqttClient = MQTTClient(CLIENT_ID, server=MQTT_BROKER, user=user, password=password, keepalive=60)
    mqttClient.connect()
    
    while True:
        buttons()
        if table >= 0:
            print(f"Publishing Table number :: {table}")
            mqttClient.publish(TOPIC, str(table).encode())
        dist = ultrasonic_distance()
        if dist <= 2:
            time.sleep(2)
            sg90.duty(30)
            time.sleep(5)
        sg90.duty(15)

    
if __name__ == "__main__":
    try:
        main()
    except OSError as e:
        print("Error: " + str(e))
        reset()
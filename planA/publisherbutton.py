import time
import ubinascii
import machine
from umqtt.simple import MQTTClient
from machine import Pin


#Set-up for IO pins based on our electronic architecture
led_A = Pin(13, Pin.OUT, value=0)    # Segment A to pin 13
led_B = Pin(12, Pin.OUT, value=0)    # Segment B to pin 12
led_C = Pin(14, Pin.OUT, value=0)    # Segment C to pin 14
led_D = Pin(27, Pin.OUT, value=0)    # Segment D to pin 27
led_E = Pin(26, Pin.OUT, value=0)    # Segment E to pin 26
led_F = Pin(25, Pin.OUT, value=0)    # Segment F to pin 25
led_G = Pin(33, Pin.OUT, value=0)    # Segment G to pin 33
led_DP = Pin(32, Pin.OUT, value=0)    # Segment DP to pin 32

push_button1 = Pin(22, Pin.IN)  # input as table 1
push_button2 = Pin(1, Pin.IN)  # input as table 2
push_button3 = Pin(3, Pin.IN)  # input as table 3
push_button4 = Pin(21, Pin.IN)  # input as table 4
push_button5 = Pin(19, Pin.IN)  # input as table 5
push_button6 = Pin(18, Pin.IN)  # input as table 6
push_button7 = Pin(5, Pin.IN)  # input as cancelOrder

MQTT_BROKER = '192.168.128.219'
CLIENT_ID = ubinascii.hexlify(machine.unique_id())
user = "roger"
password = "password"
MQTT_TOPIC = "client"

# GPIO ports for the 7seg pins
segments =  [led_A,led_B,led_C,led_D,led_E,led_F,led_G]

num = {' ':[0,0,0,0,0,0,0],
    '1':[0,0,0,0,1,1,0],
    '2':[1,1,0,1,1,0,1],
    '3':[1,0,0,1,1,1,1],
    '4':[0,0,1,0,1,1,1],
    '5':[1,0,1,1,0,1,1],
    '6':[1,1,1,1,0,1,1],
    'C':[0,0,0,1,1,1,1]}

def reset():
	print("Resetting...")
	for loop in range(0,7):
		segments[loop].value(num[' '][loop])
		time.sleep(0.001)
	time.sleep(5)
	machine.reset()

def display(n):
	for loop in range(0,7):
		segments[loop].value(num[n][loop])
		time.sleep(0.001)
	
def talker():
	Table = []
	table1_state = push_button1.value()
	table2_state = push_button2.value()
	table3_state = push_button3.value()
	table4_state = push_button4.value()
	table5_state = push_button5.value()
	table6_state = push_button6.value()
	cancel_state = push_button7.value()
	if table1_state == True:
		display('1')
		return Table + [1]
	elif table2_state == True:
		display('2')
		return Table + [2]
	elif table3_state == True:
		display('3')
		return Table + [3]
	elif table4_state == True:
		display('4')
		return Table + [4]
	elif table5_state == True:
		display('5')
		return Table + [5]
	elif table6_state == True:
		display('6')
		return Table + [6]
	elif cancel_state == True:
		display('C')
		return Table[:-1] 

def main():
	time.sleep(1)
	print(f"client {CLIENT_ID} to mqtt broker: {MQTT_BROKER}\n")
	mqttClient = MQTTClient(CLIENT_ID, 
			    server=MQTT_BROKER, 
			    user=user, 
			    password=password, 
			    keepalive=60)
	mqttClient.connect()
	print("Connected to %s, waiting for button presses" % server)

	while True:
		Table = talker()
		print(f"Publishing Table number :: {Table[-1]}")
		mqttClient.publish(MQTT_TOPIC, str(Table[-1]).encode())
		time.sleep(3))
	mqttclient.disconnect() #disconnect	
	
    
if __name__ == "__main__":
	try:
		main()
	except OSError as e:
		print("Error: " + str(e))
		reset()

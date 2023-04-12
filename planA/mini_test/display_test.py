import time
from machine import Pin

push_button1 = Pin(23, Pin.IN, Pin.PULL_UP) # input as table 1
push_button2 = Pin(22, Pin.IN, Pin.PULL_UP)  # input as table 2
push_button3 = Pin(21, Pin.IN, Pin.PULL_UP)  # input as table 3
push_button4 = Pin(19, Pin.IN, Pin.PULL_UP)  # input as table 4
push_button5 = Pin(18, Pin.IN, Pin.PULL_UP)  # input as table 5
push_button6 = Pin(5, Pin.IN, Pin.PULL_UP)  # input as table 6
push_button7 = Pin(4, Pin.IN, Pin.PULL_UP)  # input as cancelOrder

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
    elif not push_button2.value():
        display('2')
    elif not push_button3.value():
        display('3')
    elif not push_button4.value():
        display('4')
    elif not push_button5.value():
        display('5')
    elif not push_button6.value():
        display('6')
    elif not push_button7.value():
        display('C')
        
while True:           
    buttons()

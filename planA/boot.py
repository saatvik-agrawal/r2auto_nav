# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()
# boot.py -- run on boot-up
import network, utime, machine

# Replace the following with your WIFI Credentials

SSID = "eg2310_Gp6"
SSID_PASSWORD = "eg2310router"
#JG Hotspot
#SSID = "Jg"
#Remember to change this to the wifiID you are using
#SSID_PASSWORD = "jgbest11"
#Remember to change this to the wifiPassword you are using


def do_connect():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(SSID, SSID_PASSWORD)
        while not sta_if.isconnected():
            print("Attempting to connect....")
            utime.sleep(1)
    print('Connected! Network config:', sta_if.ifconfig())
    
print("Connecting to your wifi...")
do_connect()

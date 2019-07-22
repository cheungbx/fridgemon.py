# ESP8266 iot humdity temperature sensor with LED switch and pump switch , MQTT and  NTP time.
# GPIO4 aka D2 for SDA of I2C OLED SSD1306 
# GPIO5 aka D1 for SCL of I2C OLED SSD1306 
# Btn1Pin  12    // D6 - button 1 for LED 
# DHTPIN   13    // D7 data pin for the humidity sensor
# LedPin   14    // D5   LED output pin 
# Btn2Pin  2     // D4 - button 2 for pump
# PumpPin  0     // D3 - Pump output pin
import machine
import network
import time
import dht
from time import sleep
from machine import Pin, I2C
from dht import DHT11
from umqtt.robust import MQTTClient
import os
import sys
import ssd1306

ADAFRUIT_USERNAME = b'dwong'
ADAFRUIT_IO_URL = b'io.adafruit.com'
ADAFRUIT_IO_KEY = b'a52d93d0a1aa42e09850009bcb472012'
# connect to Adafruit IO MQTT broker using unsecure TCP (port 1883)
#
# To use a secure connection (encrypted) with TLS:
#   set MQTTClient initializer parameter to "ssl=True"
#   Caveat: a secure connection uses about 9k bytes of the heap
#         (about 1/4 of the micropython heap on the ESP8266 platform)
# create a random MQTT clientID
random_num = int.from_bytes(os.urandom(3), 'little')
mqtt_client_id = bytes('client_'+str(random_num), 'utf-8')


client = MQTTClient(client_id=mqtt_client_id,
                        server=ADAFRUIT_IO_URL,
                        user=ADAFRUIT_USERNAME,
                        password=ADAFRUIT_IO_KEY,ssl=False)

def connect_and_subscribe():
    global client, ADAFRUIT_USERNAME, ADAFRUIT_IO_URL, ADAFRUIT_IO_KEY
    global mqtt_feed_led, mqtt_feed_pump
    try:
        client.connect()
    except Exception as e:
        print('could not connect to MQTT server {}{}'.format(type(e).__name__, e))
        sys.exit()
    client.set_callback(cb)
    time.sleep(1)
    client.subscribe(mqtt_feed_led)
    time.sleep(1)    
    client.subscribe(mqtt_feed_pump)
    time.sleep(1)    

def fill_zero(n):   
    if n < 10:   
        return '0' + str(n) 
    else:   
        return str(n) 

def fill_blank(n):     
    if n<10:
        return ' ' + str(n)
    else:
        return str(n)
    
# callback function is called when scribed data is received from broker
def cb(topic, msg):
    global ledOn, pumpOn
    print('Received:  Topic = {}, Msg = {}\n'.format(topic, msg))
    if topic == mqtt_feed_led:        
        if msg == b'ON':
            print ('LED on')
            led.off()
            ledOn = True
        elif msg == b'OFF':
            print ('LED off')
            led.on()
            ledOn = False
    elif  topic == mqtt_feed_pump:
        if msg == b'ON':
            print ('PUMP on')
            pump.off()
            pumpOn = True
        elif msg == b'OFF':
            print ('PUMP off')
            pump.on()
            pumpOn = False

i2c = I2C(-1, Pin(5), Pin(4))   # SCL, SDA
display = ssd1306.SSD1306_I2C(128, 64, i2c)
        
# WiFi connection information
WIFI_SSID = 'BILLYWIFI'
WIFI_PASSWORD = 'Xolmem13'

# turn off the WiFi Access Point
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

# connect the device to the WiFi network
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

# wait until the device is connected to the WiFi network
MAX_ATTEMPTS = 20
attempt_count = 0
while not wifi.isconnected() and attempt_count < MAX_ATTEMPTS:
    attempt_count += 1
    time.sleep(1)

if attempt_count == MAX_ATTEMPTS:
    print('could not connect to the WiFi network')
    sys.exit()

# set time using NTP server on the internet

import ntptime    #NTP-time (from pool.ntp.org) 
import utime
ntptime.settime()
tm = utime.localtime(utime.mktime(utime.localtime()) + 8 * 3600)
tm=tm[0:3] + (0,) + tm[3:6] + (0,)
rtc = machine.RTC()
rtc.datetime(tm)
 

reported_err = 0
measure_period_ms = 10000
display_period_ms = 1000
loop_delay_ms = 10
last_measure_ms = 0
last_display_ms = 0
debounce_start_ms = 0
debounce_period_ms = 20
last_btn1_state = 0
last_btn2_state = 0
btn1_state = 0
btn2_state = 0
ledOn = False
pumpOn = False
h = 72.5
h0 = 1.0
t = 28.3
t0 = 1.0

led = Pin(14, Pin.OUT)
pump = Pin(0, Pin.OUT)
btn1 = Pin(12, Pin.IN, Pin.PULL_UP)
btn2 = Pin(2, Pin.IN, Pin.PULL_UP)
# publish humidity and airtemp to Adafruit IO using MQTT
# subscribe to the led2 feed
#
# format of feed name:
#   "ADAFRUIT_USERNAME/feeds/ADAFRUIT_IO_FEEDNAME"

mqtt_feed_led = bytes('{:s}/feeds/led'.format(ADAFRUIT_USERNAME), 'utf-8')
mqtt_feed_pump = bytes('{:s}/feeds/pump'.format(ADAFRUIT_USERNAME), 'utf-8')
mqtt_feed_humidity = bytes('{:s}/feeds/humidity'.format(ADAFRUIT_USERNAME), 'utf-8')
mqtt_feed_airtemp = bytes('{:s}/feeds/airtemp'.format(ADAFRUIT_USERNAME), 'utf-8')

connect_and_subscribe()



# pin for DHT sensor
dhtSensor = DHT11(Pin(13, Pin.IN, Pin.PULL_UP))



while True:

    # led switch
    btn1_value = btn1.value()
    if btn1_value != last_btn1_state :
        debounce_start_ms = time.ticks_ms()
    if abs(time.ticks_ms() - debounce_start_ms) >= debounce_period_ms :
        if btn1_value != btn1_state :
            btn1_state = btn1_value
            if btn1_state == 1 :
                if ledOn :
# if led is previously ON, now turn it off by outputing High voltage
                    led.on()
                    msg = "OFF"
                    ledOn = False
                else :
# if led is previously OFF, now turn it on by outputing Low voltage
                    led.off()
                    msg = "ON"
                    ledOn = True
                print('Publish:  led {}'.format(msg))
                client.publish(mqtt_feed_led, msg, qos=0)
 
    last_btn1_state = btn1_value


    # pump switch
    btn2_value = btn2.value()
    if btn2_value != last_btn2_state :
        debounce_start_ms = time.ticks_ms()
    if abs(time.ticks_ms() - debounce_start_ms) >= debounce_period_ms :
        if btn2_value != btn2_state :
            btn2_state = btn2_value
            if btn2_state == 1 :
                if pumpOn :
# if pump is previously ON, now turn it off by outputing High voltage
                    pump.on()
                    msg = "OFF"
                    pumpOn = False
                else :
# if pump is previously OFF, now turn it on by outputing Low voltage
                    pump.off()
                    msg = "ON"
                    pumpOn = True
                print('Publish:  pump {}'.format(msg))
                client.publish(mqtt_feed_pump, msg, qos=0)
    last_btn2_state = btn2_value

    # Sensors
    try:
        if abs(time.ticks_ms() - last_measure_ms) >= measure_period_ms :
            try:
                dhtSensor.measure()   # Poll sensor
                h = dhtSensor.humidity()
                t = dhtSensor.temperature()
            except OSError as err :
                print("dhtSensor error: {0}".format(err))
            msg = (b'{0:3.1f}'.format(h))
            if h != h0 :
                print('Publish:  humidity = {}'.format(h))
                client.publish(mqtt_feed_humidity, msg, qos=0)
                h0 = h

            msg = (b'{0:3.1f}'.format(t))
            if t != t0 :
                print('Publish:  airtemp = {}'.format(t))
                client.publish(mqtt_feed_airtemp, msg, qos=0)
                t0 = t
                
            last_measure_ms = time.ticks_ms()

        if abs(time.ticks_ms() - last_display_ms) >= display_period_ms :
            display.fill(0)
            Y,M,D,H,m,S,ms,W=utime.localtime()
            timetext ='%s-%s %s:%s:%s' % (fill_zero(M),fill_zero(D),fill_zero(H),fill_zero(m),fill_zero(S))       
            display.text(timetext,0,0)
            display.text(b'{0:3.1f} %'.format(h), 0, 24)
            display.text(b'{0:3.1f} C'.format(t), 64, 24)  
            if ledOn :
                display.text("LED ON", 0, 48)
            else :
                display.text("LED OFF", 0, 48)
                             
            if pumpOn :
                display.text("PUMP ON", 64, 48)
            else :
                display.text("PUMP OFF", 64, 48)

            display.show()  
            last_display_ms = time.ticks_ms()
            
    except KeyboardInterrupt:
        print('Ctrl-C pressed...exiting')
        client.disconnect()
        sys.exit()
        
        # Subscribe.  Non-blocking check for a new message.
    try:
        client.check_msg()
        time.sleep_ms(loop_delay_ms);
    except OSError: connect_and_subscribe()
    



#!/usr/bin/env python3

import apa102
import time
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from threading import Thread

led = apa102.APA102(num_led=3)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

gpio_blue = GPIO.PWM(12, 100)
gpio_red = GPIO.PWM(13, 100)
gpio_blue.start(0)
gpio_red.start(0)

def set_led(color):
    led.set_pixel(0, color[0], color[1], color[2])
    led.show()

def set_front_led(led, cycle):
    c = cycle * 0.3
    if led == "red" or led == "both":
        gpio_red.ChangeDutyCycle(c)
    if led == "blue" or led == "both":
        gpio_blue.ChangeDutyCycle(c)


def on_connect(client, userdata, flags, rc):
    print("Connected")
    client.subscribe("hermes/hotword/#")
    client.subscribe("hermes/tts/#")
    print("Callbacks added")

def on_message(client, userdata, msg):
    print (msg.payload)

def hotword_on(client, userdata, msg):
    for i in range(51):
        cycle = 255-51*5
        set_led([0,0,cycle])
        set_front_led("blue", cycle)
        time.sleep(0.005)

def hotword_off(client, userdata, msg):
    for times in range(3):
        for i in range(51):
            cycle = i*5
            set_led([0,0,cycle])
            set_front_led("blue", cycle)
            time.sleep(0.004)
        set_led([0,0,0])
    set_led([0,255,0])

def rainbow():
    global speaking_bool
    start_time = time.time()
    r = 250
    g = 125
    b = 5
    rdir = -1
    gdir = 1
    bdir = 1
    print("Rainbow started")
    while speaking_bool and time.time() - start_time < 10:
        set_led([r,g,b])
        print(r,g,b)
        if(r >= 255 or r <= 0):
            rdir = rdir * -1
        r += rdir * 5
        set_front_led("red", r)
        if(g >= 255 or g <= 0):
            gdir = gdir * -1
        g += gdir * 5
        if(b >= 255 or b <= 0):
            bdir = bdir * -1
        b += bdir *5
        set_front_led("blue", b)
        time.sleep(0.015)
    print("Rainbow ended")
    set_led([0,0,0])
    set_front_led("both", 0)

speaking_bool = False
def speaking(client, userdata, msg):
    global speaking_bool
    if speaking_bool:
        return
    speaking_bool = True
    t = Thread(target=rainbow)
    t.start()

def speaking_stop(client, userdata, msg):
    global speaking_bool
    speaking_bool = False

client = mqtt.Client()
client.on_connect = on_connect
client.message_callback_add("hermes/hotword/toggleOn", hotword_on)
client.message_callback_add("hermes/hotword/toggleOff", hotword_off)
client.message_callback_add("hermes/tts/say", speaking)
client.message_callback_add("hermes/tts/sayFinished", speaking_stop)
client.connect("localhost", 1883, 60)
set_led([0,0,0])
client.loop_forever()
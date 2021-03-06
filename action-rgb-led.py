#!/usr/bin/env python3
import apa102
import time
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import json
from threading import Thread

#########################################################################
# Dieses Skript hoert auf MQTT Nachrichten, die von Snips gesendet werden,
# und steuert damit die LEDs des Sprachassistenten
# Es soll automatisch beim Boot ausgefuehrt werden
#########################################################################

led = apa102.APA102(num_led=3)
hotword_active = True
stop_thread = False
thread_running = False

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

gpio_blue = GPIO.PWM(12, 120)
gpio_red = GPIO.PWM(13, 120)
gpio_blue.start(0)
gpio_red.start(0)

# mit dieser methode werden die LEDs auf dem ReSpeaker gesteuert
# color erwartet eine farbe in form einer liste
# pink ist z.B. [255,0,255]
# pix nimmt eine Liste der Pixel entgegen, die geaendert werden sollen (0-2)
def set_led(color, pix=[0]):
    for i in range(len(pix)):
        led.set_pixel(pix[i], color[0], color[1], color[2])
    led.show()

# mit dieser methode wird die LED gesteuert, die an GPIO 12 und 13 angeschlossen ist
def set_front_led(led, cycle):
    c = cycle * 0.3
    if led == "red" or led == "both":
        gpio_red.ChangeDutyCycle(c)
    if led == "blue" or led == "both":
        gpio_blue.ChangeDutyCycle(c)

# callback der aufgerufen wird, wenn der mqtt client verbunden ist
def on_connect(client, userdata, flags, rc):
    print("Connected")
    client.subscribe("hermes/hotword/#")
    client.subscribe("hermes/tts/#")
    client.subscribe("snips/led/#")
    client.subscribe("menu/erinnerung")
    print("Callbacks added")

# gibt jede message zu debuggingzwecken aus
def on_message(client, userdata, msg):
    print (msg.payload)

# wird aufgerufen, wenn der assistent auf das hotword wartet
def hotword_on(client, userdata, msg):
    for i in range(51):
        cycle = 255-51*5
        set_led([0,0,cycle])
        set_front_led("blue", cycle)
        time.sleep(0.005)
    global hotword_active
    hotword_active = True
    print("Hotword on")

# wird aufgerufen, wenn der assistent beginnt zuzuhoeren
def hotword_off(client, userdata, msg):
    for times in range(2):
        for i in range(51):
            cycle = i*5
            set_led([0,0,cycle])
            set_front_led("blue", cycle)
            time.sleep(0.004)
        set_led([0,0,0])
    set_led([0,0,255])
    global hotword_active
    hotword_active = False
    print("Hotword off")

# kontinuierlicher farblauf waehrend assistent spricht
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

def led_an_thread(client, userdata, msg):
    global speaking_bool
    global hotword_active
    global stop_thread
    global thread_running
    while speaking_bool or not hotword_active or thread_running:
        stop_thread = True
        time.sleep(0.1) # wait
    stop_thread = False
    thread_running = True
    c = [0,0,0]
    while c[0] < 255 and not stop_thread:
        set_led(c, [1,2])
        c[0] += 5
        c[1] += 4
        c[2] += 3
        time.sleep(0.01)
    
    # hier wird die payload decodiert
    # wenn sich ein integer darin befindet, wird dieser
    # als timer verwendet um danach die LEDs wieder abzuschalten
    try:
        load = msg.payload.decode("utf-8")
        if load != "":
            secs = float(load) * 60
            start_time = time.time()
            while c[0] > 0 and not stop_thread:
                if time.time() - start_time < secs - 30:
                    time.sleep(1)
                else:
                    set_led(c, [1,2])
                    c[0] -= 1
                    if c[1] >= 2:
                        c[1] -= 2
                    if c[2] >= 2:
                        c[2] -= 2
                    time.sleep(0.115)
            c = [0,0,0]
            set_led(c,[1,2])
    except:
        pass
    thread_running = False

def led_aus_thread(client, userdata, msg):
    global speaking_bool
    global hotword_active
    global stop_thread
    global thread_running
    while speaking_bool or not hotword_active or thread_running:
        stop_thread = True
        time.sleep(0.1) # wait
    stop_thread = False
    thread_running = True
    c = [255,204,153]
    while c[2] > 0 and not stop_thread:
        set_led(c, [1,2])
        for i in range(3):
            c[i] -= 1
        time.sleep(0.03)
    set_led([0,0,0], [1,2])
    thread_running = False

# startet den thread, der fuer das fading der LEDs dient
def led_an(client, userdata, msg):
    t = Thread(target=led_an_thread, args=(client,userdata,msg,))
    t.start()

# startet den thread, der LEDs ausfaden laesst
def led_aus(client, userdata, msg):
    t = Thread(target=led_aus_thread, args=(client,userdata,msg,))
    t.start()

speaking_bool = False
# wird aufgerufen, wenn das TTS beginnt zu sprechen
def speaking(client, userdata, msg):
    global speaking_bool
    if speaking_bool:
        return
    speaking_bool = True
    t = Thread(target=rainbow)
    t.start()

# wird aufgerufen, wenn das TTS aufhört zu sprechen
def speaking_stop(client, userdata, msg):
    global speaking_bool
    speaking_bool = False

# wenn menu/erinnerung eintrifft, erinnert dies den nutzer an die Essensbestellung
def erinnerung (client, userdata, msg):
    payload = msg.payload.decode("utf-8")
    if "snips-master" in payload:
        for i in range(3):
            cycle = 255
            while cycle > 0:
                set_front_led("red", cycle)
                cycle -= 5
                time.sleep(0.01)
        set_front_led("red", 0)
        time.sleep(2)
        msg = '{ "text" : "Bitte denke daran, dein Essen zu bestellen!", "lang" : "de", "siteId" : "default"}'
        client.publish("hermes/tts/say", msg)

client = mqtt.Client()
client.on_connect = on_connect

# diese callbacks werden aufgerufen, wenn nachrichten auf diesem topic eingehen
client.message_callback_add("hermes/hotword/toggleOn", hotword_on)
client.message_callback_add("hermes/hotword/toggleOff", hotword_off)
client.message_callback_add("hermes/tts/say", speaking)
client.message_callback_add("hermes/tts/sayFinished", speaking_stop)
client.message_callback_add("snips/led/an", led_an)
client.message_callback_add("snips/led/aus", led_aus)
client.message_callback_add("menu/erinnerung", erinnerung)
client.connect("localhost", 1883, 60)

# LEDs werden initial gesetzt
set_led([0,0,0])
set_front_led("red", 20)

# Der MQTT client hoert dauerhaft auf nachrichten
client.loop_forever()
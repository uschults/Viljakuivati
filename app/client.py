import os
import sys
import glob
import time
import read_temp
import random
import configparser
import git
import RPi.GPIO as GPIO

import traceback


from threading import Thread, Event
from subprocess import call, check_output
from paho.mqtt import client as mqtt_client
from save_data import save_to_client

#from IOPi import IOPi


# On boot all pins are input
# gpios 0-8 are pulled high, the rest are pulled low on boot

# for manual setup
# board setups
GPIO.setmode(GPIO.BOARD)
global IOPi1
global expander_bus_1, expander_bus_2

def expander_init():
    global expander_bus_1, expander_bus_2

    expander_bus_1 = IOPi1(0x20)
    expander_bus_2 = IOPi1(0x21)

    expander_bus_1.set_bus_direction(0x0000)
    expander_bus_2.set_bus_direction(0x0000)

config = configparser.ConfigParser()
config.read('configfile.ini')
#print(config.sections())


device_folders = []

##### to be moved to config file maybe
broker = '80.250.119.25'
port = 1883



# Topics in server 
# Needs to become modular
temperature_topics = ["temp1", "temp2", "temp3", "temp4", "temp5", "temp6"]

# dictionary holds motor state
# motor_topics = { motor1: [pin1, pin2]}
motor_topics = {}
level_buttons = {}
temp_sensors = {}
feedback_inputs = {}

# generate client ID with pub prefix randomly
# need to hash pw
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'urmosc'
password = 'admin'

# Variables for running automation programs
# not used atm
program_running = Event()

# ------------------------------------------------------------- #
# Inits  

def mqtt_init():
    client = connect_mqtt()
    client.loop_start()
    return client

def motor_init(motor_topics):
    #read from config file to list
    for key, value in config['MOTOR_PINS'].items():
        values_in_list = value.split(",")
        motor_topics[key] = values_in_list  # turn to int
        
        #for pin in values_in_list:
            #print(pin)
            #GPIO.setup(int(pin), GPIO.OUT) # what if it cant be cast to int
    #print("found motors:", motor_topics)

def button_init(level_buttons):
    #read from config file to list
    for key, value in config['BUTTON_PINS'].items():
        value = int(value)
        level_buttons[key] = value

    #print("found level buttons:", level_buttons)
    if(level_buttons):
        for key, value in level_buttons.items():
            # register pin as input with pulldown for raspi, pulldown
            GPIO.setup(value, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            # get initial button state
            rising_level_btn_callback(value)
            GPIO.add_event_detect(value, GPIO.BOTH, callback=rising_level_btn_callback, bouncetime=400)
    else:
        publish("debug", "No buttons")

def feedback_init(feedback_inputs):
    for key, value in config['FEEDBACK_PINS'].items():
        value = int(value)
        feedback_inputs[key] = value
         # register pin as input with pulldown for raspi
        #GPIO.setup(value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    if(feedback_inputs):
        for key, value in feedback_inputs.items():
            GPIO.setup(value, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(value, GPIO.BOTH, callback=feedback_callback, bouncetime=400)
        feedback_checks()
        return 1
    else:
        publish("debug", "No feedbacks")
        return 0

def temperature_sensor_init():
    # not needed if 1-wire interface enabled
    #os.system('modprobe w1-gpio')
    #os.system('modprobe w1-therm')

    base_dir = '/sys/bus/w1/devices/'
    device_folders = glob.glob(base_dir + '28*')
    #print(device_folders)

    found_temps = {}
    for folder in device_folders:
        found_temps[folder+ '/w1_slave'] = 1
    
    
    for key, value in config['TEMP_SENSORS'].items():
        temp_sensors[key] = value



# ------------------------------------------------------------- #
# callbacks

def rising_level_btn_callback(pin):
    #print("level pin", pin)
    # find topic releated to the pin
    for key, value in level_buttons.items():
        if value == pin:
            if(GPIO.input(pin)):
                publish(key, "true")
            else:
                publish(key, "false")
            break

def feedback_callback(pin):
    #print("feedback pin", pin)
    time.sleep(0.5)
    for key, value in feedback_inputs.items():
        if value == pin:
            if(GPIO.input(pin)):
                publish(key, "true")
            else:
                publish(key, "false")
            break

# ------------------------------------------------------------- #
# MQTT communication

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            #print("Connected to MQTT Broker!")
            publish("pistate", "Online")
            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            # maybe read directly from config??
            #publish("teade", motor_topics)
            if(motor_topics):
                for motor in motor_topics:
                    client.subscribe(motor)
            client.subscribe("check1")
            publish("debug", "subscribes done")
        else:
            with open("logfile.txt") as logfile:
                logfile.write("Failed to connect, return code %d\n", rc)
            #print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    return client

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print(msg.topic+" --  "+str(msg.payload))
    data = msg.payload.decode()
    #print(data)
    #publish("debug", data)
    #temp_topic = str(msg.topic)[0:6]
    if(msg.topic in motor_topics):
        if(data=="true"):
            motor_control(msg.topic, True)
        else:
            motor_control(msg.topic, False)

    elif(msg.topic == "check1"):
        #print("Connection check")
        publish("pistate", "Online")


def publish(topic, msg):
    global client
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        with open("logfile.txt") as logfile:
                logfile.write(f"Failed to send message to topic {topic}")
        print(f"Failed to send message to topic {topic}")

def get_temps():
    while temp_sensors:
        for topic, sensor in temp_sensors.items():
            temp = read_temp.read_temperature(sensor)
            #print(f"{sensor} : {temperature_topics[id]} :  {temp}")

            publish( topic, temp) 


            #try:
            #    save_to_client(topic, [float(temp)])
            #except Exception as error:
            #    publish("debug", str(error))
    # mayube try-except or smth needed
    publish("debug","Ei ole temperatuuriandureid")

def activate_relay_gpio(topic, state):
    GPIO.output(int(motor_topics[topic][not state]), 1)
    time.sleep(2)
    GPIO.output(int(motor_topics[topic][not state]), 0)

def activate_relay_i2c(topic, state):
    pin = int(motor_topics[topic][not state])
    if ( pin > 0 and pin < 17):
        expander_bus_1.write_pin(pin, 1)
        time.sleep(2)
        expander_bus_1.write_pin(pin, 0)
    elif ( pin > 16 and pin < 33):
        expander_bus_2.write_pin(pin, 1)
        time.sleep(2)
        expander_bus_2.write_pin(pin, 0)
    else:
        publish("debug", "wrong pin")
    publish("debug", pin)

def motor_control(topic, state):
    # toggle relay, if state = true = turn motor on
    #print("Turning motor", state)
    #print(topic, int(motor_topics[topic][not state]))
    activate_relay_thread = Thread(target = activate_relay_i2c, args= (topic, state, ))
    activate_relay_thread.start()

def feedback_checks():
    # get initial feedback state
    # cant publish before dict is ready
    for key, value in feedback_inputs.items():
        feedback_callback(value)


def main():
    global client, IOPi1
    # motor_init before mqtt or iter error
    
    client = mqtt_init()
    publish("debug", "Connection made")
    # outputs and inputs init
    try:
        motor_init(motor_topics)
        publish("debug", "motors read")
    except:
        publish("debug", "error in motor_init")

    try:
        try:
            from IOPi import IOPi as IOPi1
        except ImportError as e:
            publish("debug", str(e))
        #publish("debug", check_output(["sudo", "raspi-config", "nonint", "do_i2c", "0"]))
        #publish("debug", check_output(["sudo", "raspi-config", "nonint", "get_i2c"]))
       # publish("debug", check_output(["sudo", "i2cdetect", "-y", "1"]))
        
        expander_init()
        publish("debug", "expander setup")

    except:
        publish("debug", "error in expander_init")

    try:
        button_init(level_buttons)
        publish("debug", "buttons read")
    except: 
        publish("debug", "error in button_init")

    feedback = feedback_init(feedback_inputs)
    publish("debug", "feedbacks read")
    
    try:
        temperature_sensor_init()
        publish("debug", "temps read")
    except:
        publish("debug", "error in temp_init")
    
    # Separate thread for reading different temp sensors values
    temp_thread = Thread(target = get_temps)
    #temp_thread = Thread(target = get_temps, args=[client]) # when not using global ?
    temp_thread.start()
    
    time_last = 0
    while (True):
        if(feedback):
            time_present = time.time()
            if((time_present-time_last)>10):
                feedback_checks()
                publish("debug", "feedback_check")
                time_last=time_present
        
if __name__ == "__main__":
    #try:
    #    call(["pip", "install", "-r", "requirements.txt"])
    #except:
    #    print("error installing modules")
    try:   
        main()
    except KeyboardInterrupt:
        #print("Exiting")
        GPIO.cleanup()
        sys.exit(0)

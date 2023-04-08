import os
import sys
import glob
import time
import read_temp
import random
import configparser
import git
import RPi.GPIO as GPIO

from threading import Thread
from subprocess import call
from paho.mqtt import client as mqtt_client

#buttonpin = 11
#outputpin = 29
#outputpin2 = 31

# On boot all pins are input
# gpios 0-8 are pulled high, the rest are pulled low on boot

# for manual setup
GPIO.setmode(GPIO.BOARD)
#GPIO.setup(buttonpin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
#GPIO.setup(outputpin, GPIO.OUT)

config = configparser.ConfigParser()
config.read('configfile.ini')
print(config.sections())

gitupdater = git.cmd.Git("https://github.com/uschults/Viljakuivati.git")

device_folders = []

##### to be moved to config file maybe
broker = '80.250.119.25'
port = 1883

# Topics in server
temperature_topics = ["temp1", "temp2", "temp3"]
# dictionary holds motor state
motor_topics = {}
level_buttons = {}
temp_sensors = {}
feedback_inputs = {}

# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'urmosc'
password = 'admin'

def motor_init(motor_topics):
    #read from config file to list
    for key, value in config['MOTOR_PINS'].items():
        values_in_list = value.split(",")
        motor_topics[key] = values_in_list  # turn to int 
        for pin in values_in_list:
            print(pin)
            GPIO.setup(int(pin), GPIO.OUT) # what if it cant be cast to int
    print("found motors:", motor_topics)

def button_init(level_buttons):
    #read from config file to list
    for key, value in config['BUTTON_PINS'].items():
        value = int(value)
        level_buttons[key] = value
         # register pin as input with pulldown for raspi
        GPIO.setup(value, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # get initial button state
        rising_level_btn_callback(value)
        GPIO.add_event_detect(value, GPIO.BOTH, callback=rising_level_btn_callback, bouncetime=400)

    print("found level buttons:", level_buttons)

def feedback_init(feedback_inputs):
    for key, value in config['FEEDBACK_PINS'].items():
        value = int(value)
        feedback_inputs[key] = value
         # register pin as input with pulldown for raspi
        #GPIO.setup(value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # get initial feedback state
        feedback_callback(value)
        GPIO.add_event_detect(value, GPIO.BOTH, callback=feedback_callback, bouncetime=400)
    print("found feedbacks:", feedback_inputs)

def temperature_sensor_init():
    # not needed if 1-wire interface enabled
    #os.system('modprobe w1-gpio')
    #os.system('modprobe w1-therm')

    base_dir = '/sys/bus/w1/devices/'
    device_folders = glob.glob(base_dir + '28*')
    print(device_folders)

    for folder in device_folders:
        temp_sensors[folder+ '/w1_slave'] = 1

    #device_file = device_folder + '/w1_slave'

    ###if glob doesnt find all folders, use pref scandir or listdir. doesn't find folders with 28* yet..
    #sub_folders = [name for name in os.listdir('/sys/bus/w1/devices/2*') if os.path.isdir(os.path.join('/sys/bus/w1/devices/', name))]
    #list_subfolders_with_paths = [f.path for f in os.scandir(path) if f.is_dir()]
    #print(sub_folders)

    # old for lists
    #return device_folders

def rising_level_btn_callback(pin):
    #print("level pin", pin)
    # find topic releated to the pin
    for key, value in level_buttons.items():
        if value == pin:
            if(GPIO.input(pin)):
                publish(key, "TÄIS")
            else:
                publish(key, "tühi")
            break

def feedback_callback(pin):
    print("feedback pin", pin)
    time.sleep(0.5)
    for key, value in feedback_inputs.items():
        if value == pin:
            if(GPIO.input(pin)):
                publish(key, "OFF")
            else:
                publish(key, "ON")
            break
    
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            # maybe read directly from config??
            for motor in motor_topics:
                print(motor)
                client.subscribe(motor)
            # subscribe for update button
            #client.subscribe("update")
            # subscribe for lights
            client.subscribe("tuled1")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    return client

def motor_control(topic, state):
    # toggle relay, if state = true = turn motor on
    print("Turning motor", state)
    #print(topic, int(motor_topics[topic][not state]))
    GPIO.output(int(motor_topics[topic][not state]), 1)
    time.sleep(0.2)
    GPIO.output(int(motor_topics[topic][not state]), 0)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" --  "+str(msg.payload))
    data = msg.payload.decode()
    #print(data)
    temp_topic = str(msg.topic)[0:6]
    # if(data == "update"):
    #     print("starting update")
    #     client.loop_stop()
    #     GPIO.cleanup()
    #     msg = gitupdater.pull()
    #     print(msg)

    #     # Shouldn't restart if already up to date
    #     print("restarting") 
    #     call(["sudo", "systemctl", "restart", "kuivati.service"])

    if(temp_topic == "mootor"):
        if(data=="true"):
            motor_control(msg.topic, True)
        else:
            motor_control(msg.topic, False)
    
    elif(temp_topic == "tuled1"):
        print("switch lights")

def mqtt_init():
    client = connect_mqtt()
    client.loop_start()
    return client

def publish(topic, msg):
    global client
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def get_temps():
    while temp_sensors:
        id = 0
        for sensor in temp_sensors.keys():
            temp = read_temp.read_temperature(sensor)
            #print(f"{sensor} : {temperature_topics[id]} :  {temp}")
            publish( temperature_topics[id], temp)
            id+=1
    # mayube try-except or smth needed
    print("No temp sensors")


def main():
    global client
    client = mqtt_init()
    # outputs and inputs init
    motor_init(motor_topics)
    button_init(level_buttons)
    feedback_init(feedback_inputs)
    temperature_sensor_init()
    
    
    # Separate thread for reading different temp sensors values
    temp_thread = Thread(target = get_temps)
    #temp_thread = Thread(target = get_temps, args=[client]) # when not using global ?
    temp_thread.start()
        
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting")
        GPIO.cleanup()
        sys.exit(0)

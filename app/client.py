import os
import glob
import time
import read_temp
import random
import configparser
import git
import RPi.GPIO as GPIO

from subprocess import call
from paho.mqtt import client as mqtt_client

buttonpin = 11
outputpin = 8
GPIO.setmode(GPIO.BOARD)
GPIO.setup(buttonpin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(outputpin, GPIO.OUT)

config = configparser.ConfigParser()
config.read('configfile.ini')
print(config.sections())

gitupdater = git.cmd.Git("https://github.com/uschults/Viljakuivati.git")

device_folders = []

# to be moved to config file
broker = '80.250.119.25'
port = 1883
temperature_topic = "kuivati/temp1"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'urmosc'
password = 'admin'

#temps for testing
puuteandur_status = ""

def temperature_sensor_init():
    os.system('modprobe w1-gpio')
    os.system('modprobe w1-therm')

    base_dir = '/sys/bus/w1/devices/'
    device_folders = glob.glob(base_dir + '28*')
    print(device_folders)
    #device_file = device_folder + '/w1_slave'
    
    ###if glob doesnt find all folders, use pref scandir or listdir. doesn't find folders with 28* yet..
    #sub_folders = [name for name in os.listdir('/sys/bus/w1/devices/2*') if os.path.isdir(os.path.join('/sys/bus/w1/devices/', name))]
    #list_subfolders_with_paths = [f.path for f in os.scandir(path) if f.is_dir()]

    #print(sub_folders)

    return device_folders

    
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
             # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            for key in config['MOTOR_PINS']:
                print(f"mootor/{key}")
                client.subscribe(f"mootor/{key}")
            client.subscribe("update")
                
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    return client

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" --  "+str(msg.payload))
    data = msg.payload.decode()
    print(data)
    if(msg.topic == "mootor/mootor1"):
        if(data=="true"):
            GPIO.output(outputpin, 0)
            print("turn motor on")
        else:
            GPIO.output(outputpin, 1)
            print("Turn motor off")
    elif(data == "update"):
        print("starting update")
        client.loop_stop()
        msg = gitupdater.pull()
        print(msg)
        call(["sudo", "systemctl", "restart", "kuivati.service"])
        print("restarting")

def mqtt_init():
    client = connect_mqtt()
    client.loop_start()
    return client

def publish(client, topic, msg ):
    
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def get_temp():
    time.sleep(1)
    temp = read_temp.read_temperature(device_folders[0]+ '/w1_slave')
    return temp

def main(client):
    while True:
        #get temp and send to server
        msg = get_temp()
        publish(client, temperature_topic, msg)


        if(GPIO.input(buttonpin)):
            puuteandur_status = "high"
            print(puuteandur_status)
            GPIO.output(outputpin, 0)
        else:
            puuteandur_status = "low"
            print(puuteandur_status)
            GPIO.output(outputpin, 1)
        publish(client, "puuteandur/punker", puuteandur_status)
        

if __name__ == "__main__":
    #mqtt init
    client = mqtt_init()
    #auto detect temperature sensors and save
    device_folders = temperature_sensor_init()
    #publish(client)
    main(client)
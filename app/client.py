import os
import glob
import time
import read_temp
import random
import configparser

from paho.mqtt import client as mqtt_client

config = configparser.ConfigParser()
config.read('configfile.ini')
print(config.sections())

device_folders = []

# to be moved to config file
broker = '80.250.119.25'
port = 1883
topic = "kuivati/temp1"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'urmosc'
password = 'admin'


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

def mqtt_init():
    client = connect_mqtt()
    client.loop_start()
    return client

def publish(client):
    time.sleep(1)
    msg = read_temp.read_temperature(device_folders[0]+ '/w1_slave')
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def main(client):
    while True:
        publish(client)


if __name__ == "__main__":
    #mqtt init
    client = mqtt_init()
    #auto detect temperature sensors and save
    device_folders = temperature_sensor_init()
    #publish(client)
    main(client)
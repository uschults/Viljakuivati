import os
import glob
import time
import read_temp
import random

from paho.mqtt import client as mqtt_client


device_folders = []


def temperature_sensor_init():
    os.system('modprobe w1-gpio')
    os.system('modprobe w1-therm')

    base_dir = '/sys/bus/w1/devices/'
    device_folder = glob.glob(base_dir + '28*')[0]
    device_file = device_folder + '/w1_slave'
    
    sub_folders = [name for name in os.listdir('/sys/bus/w1/devices/') if os.path.isdir(os.path.join('/sys/bus/w1/devices/', name))]
    #list_subfolders_with_paths = [f.path for f in os.scandir(path) if f.is_dir()]

    print(sub_folders)

    

    return 0

    
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def mqtt_init():
    client = connect_mqtt()
    #cl

def main():

    read_temp.read_temperature()
    return 0



if __name__ == "__main__":
   # mqtt_init()
    temperature_sensor_init()

   # main()
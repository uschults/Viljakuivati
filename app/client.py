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
    
    sub_folders = [name for name in os.listdir('/sys/bus/w1/devices/28*') if os.path.isdir(os.path.join('/sys/bus/w1/devices/28*', name))]
    print(sub_folders)

    

    return 0


def main():

    read_temp.read_temperature()
    return 0



if __name__ == "__main__":
    temperature_sensor_init()
    main()
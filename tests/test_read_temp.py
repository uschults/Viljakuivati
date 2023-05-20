import glob

datastring = ''
while True:
    for sensor in glob.glob("/sys/bus/w1/devices/28-00*/w1_slave"):
        id = sensor.split("/")[5]
        try:
            f = open(sensor, "r")
            data = f.read()
            f.close()
            if "YES" in data:
                (discard, sep, reading) = data.partition(' t=')
                t = float(reading) / 1000.0    #reports temperature in degrees C
                datastring = datastring + id + ':' + str(t) + ','
                print(datastring)

        except:
            print("error")    
            pass
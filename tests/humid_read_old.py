def get_humid():
    global DHT_SENSOR1, DHT_SENSOR2, Adafruit_DHT
    try:
        DHT_SENSOR1 = Adafruit_DHT.DHT22
        DHT_SENSOR2 = Adafruit_DHT.DHT22
        sensors = [DHT_SENSOR1, DHT_SENSOR2]
    except Exception as e:
        publish("debug", str(e))
        publish("debug", "ERROR: can't create dht22 object")
        return 0

    while humid_sensors:
        try:
            id = 0
            for key, value in humid_sensors.items():
                humidity, temperature = Adafruit_DHT.read_retry(sensors[id], value)
                time.sleep(0.2)
                if humidity is not None and temperature is not None:
                    humid_value = "Temp={0:0.1f}*C  Humidity={1:0.1f}%".format(temperature, humidity)
                    publish(key, humid_value)
                    publish("debug", key+ humid_value)
                else:
                    publish("debug","ERROR: Failed to retrieve data from humidity sensor")
                    publish("debug", str(humidity))
                id+=1
        except Exception as e:
            publish("debug", "ERROR: reading humid sensor")
            publish("debug", str(e))
            return 0
        time.sleep(3)

def get_humid2():
    global pigpio, DHT22
    INTERVAL = 3
    try:
        pi = pigpio.pi()
    except Exception as e:  
        publish("debug", "piobject" + str(e))
        return 0

    try:
        sensor1 = DHT22.sensor(pi,17)
        sensor2 = DHT22.sensor(pi,27)
        next_reading = time.time()
        sensors =[sensor1,sensor2]
    except Exception as e:  
        publish("debug", "sensorobject" + str(e))
        return 0

    while humid_sensors:
        try:
            id = 0
            for key, value in humid_sensors.items():
                sensors[id].trigger()
                time.sleep(0.2)
                publish(key, "{}".format(sensors[id].humidity()))
                publish("debug", "{} {} {:3.2f} {} {} {} {}".format(sensors[id].humidity(), sensors[id].temperature(), sensors[id].staleness(),sensors[id].bad_checksum(), sensors[id].short_message(), sensors[id].missing_message(),sensors[id].sensor_resets()))
                
                id+=1
            next_reading+=INTERVAL
            time.sleep(next_reading-time.time())

        except Exception as e:
            pi.stop()
            publish("debug", "ERROR: reading humid sensor")
            publish("debug", str(e))
            return 0

diction = {"aaa": "sens1", "bbb":"sens2"}

sensor_errors = {}
for topic, sensor in diction.items():
    sensor_errors[sensor] = 0

print(diction)
print(sensor_errors)


sensor_errors["sens1"] = 2
print(sensor_errors)

sensor_errors["sens1"] = sensor_errors["sens1"] -1

print(sensor_errors)

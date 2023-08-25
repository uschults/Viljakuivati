import configparser
config = configparser.ConfigParser()

print(config.sections())
print(config.read('configfile.ini'))
print(config.sections())

print('TEMP_SENSORS' in config)

for key, value in config["MOTOR_PINS"].items():
    print(key, value)
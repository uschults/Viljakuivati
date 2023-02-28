import configparser
config = configparser.ConfigParser()

config.read('configfile.ini')
print(config.sections())

for key in config['MOTOR_PINS']:
    print(key)
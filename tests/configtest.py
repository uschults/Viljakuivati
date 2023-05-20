import configparser
config = configparser.ConfigParser()

print(config.sections())
config.read('configfile.ini')
print(config.sections())

print('MOTOR_PINS' in config)

for key, value in config['MOTOR_PINS'].items():
    print(key, value)
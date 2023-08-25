from configparser import ConfigParser
config_object = ConfigParser()
# all pins indicate physical pin number
# pins for buttons

config_object['MOTOR_PINS'] = {
    'mootor1' : '29',
    'mootor2' : '31',
    'mootor3' : '33',
    'mootor4' : '35'
}

#Write the above sections to config.ini file
with open('configfile.ini', 'w') as conf:
    config_object.write(conf)
    
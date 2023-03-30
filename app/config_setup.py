from configparser import ConfigParser
config_object = ConfigParser()
# all pins indicate physical pin number
# pins for buttons
config_object['BUTTON_PINS'] = {
    "btn1" : "16",
    "btn2" : "18"
}

# pins for temp sensor
# uses onewire protocol
config_object['TEMP_PINS'] = {
    "temp_sens" : "7"
}

config_object['MOTOR_PINS'] = {
    'mootor1' : '29',
    'mootor2' : '31',
    'mootor3' : '33',
    'mootor4' : '35'
}

config_object['SETUP'] = {
    'mqtt_server_ip' : '80.250.119.25',
    'port' : '1883',
    'username' : 'urmosc',
    'password' : 'admin'
}

#Write the above sections to config.ini file
with open('configfile.ini', 'w') as conf:
    config_object.write(conf)
    
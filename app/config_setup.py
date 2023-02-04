from configparser import ConfigParser
config_object = ConfigParser()
# all pins indicate physical pin number
# pins for buttons
config_object["BUTTON_PINS"] = {
    "btn1" : "16",
    "btn2" : "18"
}

# pins for temp sensor
# uses onewire protocol
config_object["TEMP_PINS"] = {
    "temp_sens" : "7"
}

#Write the above sections to config.ini file
with open('config.ini', 'w') as conf:
    config_object.write(conf)
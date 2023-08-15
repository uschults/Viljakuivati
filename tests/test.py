found = {"/sys/bus/w1/devices/28-3ce104579b67/w1_slave":1,"/sys/bus/w1/devices/28-062018797bc2/w1_slave":1}
temporary = {"temp1" : "/sys/bus/w1/devices/28-3ce104579b67/w1_slave", "temp2" : "28-062018797bc2"}


if("/sys/bus/w1/devices/28-3ce104579b67/w1_slave" in temporary.values()):
    print("yes")
else:
    print("no")

print("temp1")
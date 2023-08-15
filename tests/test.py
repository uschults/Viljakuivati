import datetime
found = {"/sys/bus/w1/devices/28-3ce104579b67/w1_slave":1,"/sys/bus/w1/devices/28-062018797bc2/w1_slave":1}
temporary = {"temp1" : "/sys/bus/w1/devices/28-3ce104579b67/w1_slave", "temp2" : "28-062018797bc2"}


if("/sys/bus/w1/devices/28-3ce104579b67/w1_slave" in temporary.values()):
    print("yes")
else:
    print("no")

file_name = "test"
print("home/pi/" + file_name + ".csv")


a ="15.22"
b = float(a)
time = datetime.datetime.now()
c = [time ,b]
print(time)
print(c)
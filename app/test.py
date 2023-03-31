
dic = {"motor1": [10, 20],"motor2":[2,1]}
state = False
for motor, value in dic.items():
    print(motor, value[not state], value[True])

#for key, value in config['MOTOR_PINS'].items():
#    print(key, value)

print("terekest"[0:4])
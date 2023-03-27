dic = {"motor1":1,"motor2":2}

for motor in dic.keys():
    print(motor)

for motor in dic.values():
    print(motor)

for motor in dic:
    print(dic[motor])

if dic:
    print("yes")
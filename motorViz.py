import matplotlib.pyplot as plt
file = open('motorOuts', 'r')


leftMotor = []
rightMotor = []

for num, line in enumerate(file):
    data = line.strip().split(" ")
    # print(data)
    leftMotor.append(int(data[0]))
    rightMotor.append(int(data[1]))
fig = plt.figure()
plt.plot(leftMotor, 'r', label = "Left Motor")
plt.plot(rightMotor, 'g', label = "Right Motor")
plt.show()

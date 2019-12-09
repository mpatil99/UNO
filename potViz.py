import matplotlib.pyplot as plt
file = open('potOut.txt', 'r')
plt.close()

pot = []
for num, line in enumerate(file):
    data = line.strip().split(" ")
    # print(data)
    pot.append(int(data[0]))
    if num % 1000 == 0:
        print(num)

plt.plot(pot, label = "Pot")
plt.show()
print(max(pot))
print(min(pot))

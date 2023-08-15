import matplotlib.pyplot as plt
import numpy as np

file = '/home/jardeldyonisio/lognav_ws/src/turtlebot3_teleop_recorder/scripts/recorded_before.txt'
SAMPLE_RATIO = 50
length_before = round(sum(1 for _ in open(file)) / 100, 0)
print(length_before)
with open(file, 'r') as f:
    x_before = []
    y_before= []
    length = sum(1 for _ in open(file))
    for index, line in enumerate(f):
        if index % SAMPLE_RATIO == 0:
            x0, y0 = line.split(',')
            x_before.append(float(x0))
            y_before.append(float(y0))
    x_before= np.array(x_before)
    y_before= np.array(y_before)
print(x_before)

file = '/home/jardeldyonisio/lognav_ws/src/turtlebot3_teleop_recorder/data/recorded_after.txt'
length_after = sum(1 for _ in open(file))

ratio_lengths = round(length_after / length_before, 0)
print(ratio_lengths)
with open(file, 'r') as f:    
    x_after = []
    y_after = []
    for index, line in enumerate(f):
        if index % ratio_lengths == 0:
            x0, y0 = line.split(',')
            x_after.append(float(x0))
            y_after.append(float(y0))
    x_after = np.array(x_after)
    y_after = np.array(y_after)
print(x_after)

plt.title('Comparação entre curva gravado e curva executada')
plt.plot(x_before, y_before, marker='^', label='Gravado')
plt.plot(x_after, y_after, marker='o', label='Executado')
plt.legend()
plt.show()
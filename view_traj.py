import sys
import numpy as np
import matplotlib.pyplot as plt
f = open("Datasets/consistent_dataset.txt")

lines = f.read().splitlines()

x = []
y = []
x_m = []
y_m = []
for l in lines:
	tokens = l.split(":")
	tracker_pose = tokens[-1].strip()
	xy = tracker_pose.split(" ")
	x.append(float(xy[0]))
	y.append(float(xy[1]))
	model_pose = tokens[3].strip()
	xy = model_pose.split(" ")
	x_m.append(float(xy[0]))
	y_m.append(float(xy[1]))

plt.plot(x, y)
#plt.plot(x_m, y_m)
plt.show()

import sys
import numpy as np
import matplotlib.pyplot as plt
f = open("Datasets/dataset.txt")

lines = f.read().splitlines()

x = []
y = []
x_m = []
y_m = []
c = 0
for l in lines:
	c+=1
	if(c < 9):
		continue
	tokens = l.split(":")
	tracker_pose = tokens[-1].strip()
	xy = tracker_pose.split(" ")
	x.append(float(xy[0]))
	y.append(float(xy[1]))
	# model_pose = tokens[3].strip()
	# xy = model_pose.split(" ")
	# x_m.append(float(xy[0]))
	# y_m.append(float(xy[1]))

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x, y)
# ax.scatter(x_m, y_m)
# for i in range(0, len(x)):
# 	ax.annotate(i, (x[i], y[i]))
ax.axis("equal")
plt.plot(x, y)
# plt.plot(x_m, y_m)
plt.show()

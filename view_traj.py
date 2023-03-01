import sys
import numpy as np
import matplotlib.pyplot as plt
f = open("dataset/clened_dataset_from_spaces.txt")

lines = f.read().splitlines()

c = 0
x = []
y = []
x_m = []
y_m = []
for l in lines:
	c += 1
	if(c < 10):
		continue
	tokens = l.split(":")
	tracker_pose = tokens[-1].strip()
	xy = tracker_pose.split(" ")
	x.append(float(xy[0]))
	y.append(float(xy[1]))
	model_pose = tokens[-2].strip()
	xy_m = model_pose.split(" ")
	x_m.append(float(xy_m[0]))
	y_m.append(float(xy_m[1]))

x_np = np.asarray(x)
y_np = np.asarray(y)
x_m_np = np.asarray(x_m)
y_m_np = np.asarray(y_m)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x, y)
ax.scatter(x_m, y_m)
# for i in range(0, len(x)):
# 	ax.annotate(i, (x[i], y[i]))
ax.axis("equal")
plt.plot(x, y)
plt.plot(x_m, y_m)
plt.show()

import sys
import numpy as np
import matplotlib.pyplot as plt
f = open("dataset.txt")

lines = f.read().splitlines()

c = 0
x = []
y = []
for l in lines:
	c += 1
	if(c < 10):
		continue
	if(c == 35):
		break
	tokens = l.split(":")
	tracker_pose = tokens[-1].strip()
	xy = tracker_pose.split(" ")
	x.append(float(xy[0]))
	y.append(float(xy[1]))

x_np = np.asarray(x)
y_np = np.asarray(y)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x, y)
# for i in range(0, len(x)):
# 	ax.annotate(i, (x[i], y[i]))
ax.axis("equal")
plt.plot(x, y)
plt.show()

import numpy as np

data = np.genfromtxt('dataset/consistent_dataset.txt',
                     dtype = float,
                     delimiter=' ')
time = data[:, 0]
ticks = data[:, 1:3]
model_pose = data[:, 3:6]
tracker_pose = data[:, 6:9]
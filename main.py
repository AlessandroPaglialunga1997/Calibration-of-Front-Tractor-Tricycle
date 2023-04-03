
import matplotlib.pyplot as plt
def plot_trajectory(x, y):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(x, y)
    ax.axis("equal")
    plt.plot(x, y)
    plt.show()

# Ks = Ksteer, Kt = Ktraction 


# IncEnc = i-th and (i+1)-th Incremental Encoder Informations
# AbsEnc = i-th and (i+1)-th Absolute Encoder Informations
import math
def prediction(current_model_pose, parameters, IncEnc, AbsEnc):
    Ks, Kt, axis_length, steer_off = [parameters[0], parameters[1], parameters[2], parameters[3]]
    DeltaIncEnc = IncEnc[1] - IncEnc[0]
    curr_x = current_model_pose[0]
    curr_y = current_model_pose[1]
    curr_theta = current_model_pose[2]
    curr_phi = Ks*AbsEnc[0] - steer_off*math.pi/180
    cos_theta_phi = math.cos(curr_theta + curr_phi)
    sin_theta_phi = math.sin(curr_theta + curr_phi)
    sin_phi = math.sin(curr_phi)
    new_x = curr_x + cos_theta_phi*Kt*DeltaIncEnc
    new_y = curr_y + sin_theta_phi*Kt*DeltaIncEnc
    new_theta = curr_theta + sin_phi*Kt*DeltaIncEnc/axis_length
    return [new_x, new_y, new_theta]

def front_rear_position(x, y, theta, axis_length, switch):
    new_x = x + switch*axis_length*math.cos(theta)
    new_y = y + switch*axis_length*math.sin(theta)
    return [new_x, new_y, theta]

# compute trajectory given parameter values and encoders informations
def compute_trajectory(parameters, IncEnc_array, AbsEnc_array):
    Ks, Kt, axis_length, steer_off = [parameters[0], parameters[1], parameters[2], parameters[3]]
    num_timestamp = len(IncEnc_array)
    model_pose = []
    current_model_pose = [0, 0, 0]
    for i in range(num_timestamp-1):
        IncEnc = [IncEnc_array[i], IncEnc_array[i+1]]
        AbsEnc = [AbsEnc_array[i], AbsEnc_array[i+1]]
        curr_front_model_pose = front_rear_position(current_model_pose[0], current_model_pose[1], current_model_pose[2], axis_length, 1)
        new_front_model_pose = prediction(curr_front_model_pose, parameters, IncEnc, AbsEnc)
        new_model_pose = front_rear_position(new_front_model_pose[0], new_front_model_pose[1], new_front_model_pose[2], axis_length, -1)
        model_pose.append(new_model_pose)
        current_model_pose = new_model_pose
    return np.asarray(model_pose)

def error(state, measurement, solution, IncEnc, AbsEnc):
    new_front_model_pose = prediction(state, solution, IncEnc, AbsEnc)
    pose_prediction = front_rear_position(new_front_model_pose[0], new_front_model_pose[1], new_front_model_pose[2], solution[2], -1)
    return [pose_prediction, pose_prediction - measurement]

def Jacobian(state, solution, IncEnc, AbsEnc):
    J = np.zeros((3, 4))
    dx = np.zeros(4)
    epsilon = 1e-4
    for i in range(len(dx)):
        dx[i] = epsilon
        first_prediction = np.array(prediction(state, solution + dx, IncEnc, AbsEnc))
        second_prediction = np.array(prediction(state, solution - dx, IncEnc, AbsEnc))
        J[:, i] = first_prediction - second_prediction
        dx[i] = 0
    J *= 0.5/epsilon
    return J

from numpy import linalg
def ls_calibrate_odometry(initial_state, measurements, initial_solution, IncEnc_array, AbsEnc_array):
    H = np.zeros((4, 4))
    b = np.zeros((4, 1))
    dx = np.zeros((4, 1))
    front_initial_state = front_rear_position(initial_state[0], initial_state[1], initial_state[2], initial_solution[2], 1)
    for i in range(len(measurements) - 1):
        IncEnc = [IncEnc_array[i], IncEnc_array[i+1]]
        AbsEnc = [AbsEnc_array[i], AbsEnc_array[i+1]]
        z = measurements[i+1, :]
        pose_prediction, e = error(front_initial_state, z, initial_solution, IncEnc, AbsEnc)
        e = np.reshape(e, (3, 1))
        J = Jacobian(front_initial_state, initial_solution, IncEnc, AbsEnc)
        H += np.matmul(np.transpose(J), J)
        b += np.matmul(np.transpose(J), e)
        front_initial_state = front_rear_position(pose_prediction[0], pose_prediction[1], pose_prediction[2], initial_solution[2], 1)
    dx = -np.matmul(linalg.pinv(H), b)
    return initial_solution + np.transpose(dx)
    


import numpy as np
data = np.genfromtxt('dataset/consistent_dataset.txt',
                     dtype = float,
                     delimiter=' ')
time = data[:, 0]
ticks = data[:, 1:3]
model_pose = data[:, 3:6]
tracker_pose = data[:, 6:9]
# Ground Truth trajectory
# plot_trajectory(model_pose[:, 0], model_pose[:, 1])

#                           [Ksteer, Ktraction, axis_length, steer_offset ]
# [0.1,    0.0106141, 1.4,         0]
initial_solution = np.array([math.pi*0.1/8192, 5, -0.5, -20])
AbsEnc_array = ticks[:, 0]
IncEnc_array = ticks[:, 1]

# Uncalibrated Odometry
# uncalibrated_model_pose = compute_trajectory(initial_solution, IncEnc_array, AbsEnc_array)
# plot_trajectory(uncalibrated_model_pose[:, 0], uncalibrated_model_pose[:, 1])

# Calibration Parameters
calibrated_parameters = initial_solution
for i in range(5):
    print(i)
    calibrated_parameters = ls_calibrate_odometry(model_pose[0, :], model_pose, calibrated_parameters, IncEnc_array, AbsEnc_array)
    calibrated_parameters = np.reshape(calibrated_parameters, (4))
print(calibrated_parameters)
calibrated_model_pose = compute_trajectory(calibrated_parameters, IncEnc_array, AbsEnc_array)
# plot_trajectory(calibrated_model_pose[:, 0], calibrated_model_pose[:, 1])

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(calibrated_model_pose[0:100, 0], calibrated_model_pose[0:100, 1])
ax.scatter(model_pose[0:100, 0], model_pose[0:100, 1])
# for i in range(0, len(x)):
# 	ax.annotate(i, (x[i], y[i]))
ax.axis("equal")
plt.show()
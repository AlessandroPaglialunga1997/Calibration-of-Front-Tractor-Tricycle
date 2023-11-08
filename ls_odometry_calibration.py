from utility import *
from front_tractor_tricycle import *
import numpy as np

def ls_calibrate_odometry(kinematic_parameters, 
                          measurements, 
                          predicted_laser_odometry,
                          init_front_pose,
                          encoders_values, 
                          max_enc_values,
                          first_sample_idx,
                          epsilon):
    batch_size = measurements.shape[0]
    parameters_num = kinematic_parameters.shape[0]
    perturbation = np.zeros((parameters_num))
    laser_odometries_added_dx = []
    laser_odometries_subtracted_dx = []
    for j in range(parameters_num):
        perturbation[j] = epsilon
        front_wheel_odometry_added_dx = compute_front_wheel_odometry(init_front_pose, + perturbation + kinematic_parameters, encoders_values, max_enc_values)
        front_wheel_odometry_subtracted_dx = compute_front_wheel_odometry(init_front_pose, - perturbation + kinematic_parameters, encoders_values, max_enc_values)
        laser_odometry_added_dx = compute_laser_odometry(+ perturbation + kinematic_parameters, front_wheel_odometry_added_dx)
        laser_odometry_subtracted_dx = compute_laser_odometry(- perturbation + kinematic_parameters, front_wheel_odometry_subtracted_dx)
        perturbation[j] = 0
        laser_odometries_added_dx.append(laser_odometry_added_dx)
        laser_odometries_subtracted_dx.append(laser_odometry_subtracted_dx)
    laser_odometries_added_dx = np.array(laser_odometries_added_dx)
    laser_odometries_subtracted_dx = np.array(laser_odometries_subtracted_dx)
    dx = np.zeros(kinematic_parameters.shape[0])
    H = np.zeros((parameters_num,parameters_num))
    b = np.zeros((parameters_num,1))
    error_array = []
    z_array = []
    h_x_array = []
    difference_array = []
    chi = 0
    acc_error = 0
    for i in range(0, measurements.shape[0]):
        error = np.zeros((3,1))
        h_x = predicted_laser_odometry[i, :]
        h_x_array.append(h_x)
        z = measurements[i, :]
        z_array.append(z)
        error[0:2, 0] = h_x[0:2] - z[0:2]
        error[2, 0] = difference_btw_angles(h_x[2], z[2])
        error_array.append(error)
        acc_error += error[0]*error[0] + error[1]*error[1] + error[2]*error[2] 
        chi += error[0]*error[0] + error[1]*error[1] + error[2]*error[2]
        J = Jacobian(laser_odometries_added_dx, laser_odometries_subtracted_dx, i+first_sample_idx, parameters_num, epsilon)
        H += np.matmul(np.transpose(J), J)
        b += np.matmul(np.transpose(J), error)
    dx = np.zeros((parameters_num,1))
    dx = -np.matmul(np.linalg.pinv(H), b)
    print(acc_error)
    return kinematic_parameters + np.reshape(dx, (parameters_num))

#--------------------------------------------------------------------------------------------

def Jacobian(laser_odometries_added_dx, laser_odometries_subtracted_dx, idx, parameters_num, epsilon):
    J = np.zeros((3, parameters_num))
    for i in range(parameters_num):
        plus_dx = laser_odometries_added_dx[i, idx, :]
        minus_dx = laser_odometries_subtracted_dx[i, idx, :]
        J[0:2, i] = plus_dx[0:2] - minus_dx[0:2]
        J[2, i] = difference_btw_angles(plus_dx[2], minus_dx[2])
    J *= 0.5/epsilon
    return J


#--------------------------------------------------------------------------------------------

# def ls_calibrate_odometry(kinematic_parameters, 
#                           measurements, 
#                           predicted_laser_odometry, 
#                           laser_odometries_added_dx, 
#                           laser_odometries_subtracted_dx,
#                           first_sample_idx):
#     parameters_num = kinematic_parameters.shape[0]
#     H = np.zeros((parameters_num,parameters_num))
#     b = np.zeros((parameters_num,1))
#     error_array = []
#     z_array = []
#     h_x_array = []
#     difference_array = []
#     chi = 0
#     for i in range(0, measurements.shape[0]):
#         error = np.zeros((3,1))
#         h_x = predicted_laser_odometry[i, :]
#         h_x_array.append(h_x)
#         z = measurements[i, :]
#         z_array.append(z)
#         error[0:2, 0] = h_x[0:2] - z[0:2]
#         error[2, 0] = difference_btw_angles(h_x[2], z[2])
#         error_array.append(error)
#         chi += error[0]*error[0] + error[1]*error[1] + error[2]*error[2]
#         J = Jacobian(laser_odometries_added_dx, laser_odometries_subtracted_dx, i+first_sample_idx, parameters_num)
#         H += np.matmul(np.transpose(J), J)
#         b += np.matmul(np.transpose(J), error)
#     dx = np.zeros((parameters_num,1))
#     dx = -np.matmul(np.linalg.pinv(H), b)
#     return np.array(h_x_array), np.array(z_array), np.array(error_array), kinematic_parameters + np.reshape(dx, (parameters_num)), chi

# #--------------------------------------------------------------------------------------------

# def Jacobian(laser_odometries_added_dx, laser_odometries_subtracted_dx, idx, parameters_num):
#     J = np.zeros((3, parameters_num))
#     epsilon = 1e-4
#     for i in range(laser_odometries_added_dx.shape[0]):
#         plus_dx = laser_odometries_added_dx[i, idx, :]
#         minus_dx = laser_odometries_subtracted_dx[i, idx, :]
#         J[0:2, i] = plus_dx[0:2] - minus_dx[0:2]
#         J[2, i] = difference_btw_angles(plus_dx[2], minus_dx[2])
#     J *= 0.5/epsilon
#     return J

#--------------------------------------------------------------------------------------------



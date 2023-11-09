# IMPORTANT NOTE           [0]  [1]      [2]          [3]           [4]             [5]               [6]
# kinematic parameters = [ Ks | Kt | axis_length | steer_off | robot_x_laser | robot_y_laser | robot_theta_laser]

#--------------------------------------------------------------------------------------------

from utility import *
from front_tractor_tricycle import *
from graphic_representation import *
import numpy as np

#--------------------------------------------------------------------------------------------
# To measure numerical Jacobian perturbated trajectories are needed.
# Given perurbeted initial front wheel config., kinematic parameters by epsilon and encoders measurements,
# return two laser wheel odometry trajectory: one with kinematic parameters perturbated by [-]epsilon
# and one with kinematic parameters perturbated by [+]epsilon

def compute_perturbed_trajectories(epsilon, init_front_pose, kinematic_parameters, encoders_values, max_enc_values):
    laser_odometries_added_dx = []
    laser_odometries_subtracted_dx = []
    parameters_num = kinematic_parameters.shape[0]
    perturbation = np.zeros((parameters_num))
    # For each kinematic parameters compute perturbated laser odometry trajectories
    # perturbing the current parameter and leaving the others at zero
    for j in range(parameters_num):
        perturbation[j] = epsilon
        # Compute perturbated front wheel odometry trajectories
        front_wheel_odometry_added_dx = compute_front_wheel_odometry(init_front_pose, + perturbation + kinematic_parameters, encoders_values, max_enc_values)
        front_wheel_odometry_subtracted_dx = compute_front_wheel_odometry(init_front_pose, - perturbation + kinematic_parameters, encoders_values, max_enc_values)
        # Compute perturbated laser odometry trajectories
        laser_odometry_added_dx = compute_laser_odometry(+ perturbation + kinematic_parameters, front_wheel_odometry_added_dx)
        laser_odometry_subtracted_dx = compute_laser_odometry(- perturbation + kinematic_parameters, front_wheel_odometry_subtracted_dx)
        
        perturbation[j] = 0    
        laser_odometries_added_dx.append(laser_odometry_added_dx)
        laser_odometries_subtracted_dx.append(laser_odometry_subtracted_dx)
    
    laser_odometries_added_dx = np.array(laser_odometries_added_dx)
    laser_odometries_subtracted_dx = np.array(laser_odometries_subtracted_dx)
    return laser_odometries_added_dx, laser_odometries_subtracted_dx
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

def ls_calibrate_odometry(kinematic_parameters, measurements, 
                          predicted_laser_odometry, init_front_pose,
                          encoders_values, max_enc_values,
                          first_sample_idx, epsilon):

    batch_size = measurements.shape[0]
    parameters_num = kinematic_parameters.shape[0]    
    dx = np.zeros(kinematic_parameters.shape[0])
    H = np.zeros((parameters_num,parameters_num))
    b = np.zeros((parameters_num,1))
    chi = 0

    laser_odometries_added_dx, laser_odometries_subtracted_dx = compute_perturbed_trajectories(epsilon, init_front_pose, kinematic_parameters, encoders_values, max_enc_values)

    for i in range(0, measurements.shape[0]):
        error = np.zeros((3,1))
        h_x = predicted_laser_odometry[i, :]
        z = measurements[i, :]
        error[0:2, 0] = h_x[0:2] - z[0:2]
        error[2, 0] = difference_btw_angles(h_x[2], z[2])
        J = Jacobian(laser_odometries_added_dx, laser_odometries_subtracted_dx, i+first_sample_idx, parameters_num, epsilon)
        H += np.matmul(np.transpose(J), J)
        b += np.matmul(np.transpose(J), error)

        chi += np.matmul(np.transpose(error), error)

    dx = -np.matmul(np.linalg.pinv(H), b)
    print(chi)
    return kinematic_parameters + np.reshape(dx, (parameters_num))

#--------------------------------------------------------------------------------------------

def fit(epsilon, batch_size, batches_number, rounds_number_batch, ax, 
        laser_odometry, kinematic_parameters, init_front_pose, 
        encoders_values, max_enc_values, 
        predicted_xy_laser_plot, predicted_theta_laser_plot):

    for round_idx in range(rounds_number_batch):
        for batch_idx in range(0, batches_number):
            predicted_front_wheel_odometry = compute_front_wheel_odometry(init_front_pose, kinematic_parameters, encoders_values[:, :], max_enc_values)
            predicted_laser_odometry = compute_laser_odometry(kinematic_parameters, predicted_front_wheel_odometry)
            double_plot(ax, predicted_laser_odometry, predicted_xy_laser_plot, predicted_theta_laser_plot)
            
            first_sample_idx = batch_idx*batch_size
            last_sample_idx = (batch_idx+1)*batch_size
            last_sample_idx = last_sample_idx if last_sample_idx < laser_odometry.shape[0] else laser_odometry.shape[0]
            
            kinematic_parameters = ls_calibrate_odometry(kinematic_parameters, laser_odometry[first_sample_idx:last_sample_idx, :], predicted_laser_odometry[first_sample_idx:last_sample_idx, :], init_front_pose, encoders_values[:last_sample_idx, :], max_enc_values, first_sample_idx, epsilon)
            
    return kinematic_parameters

#--------------------------------------------------------------------------------------------

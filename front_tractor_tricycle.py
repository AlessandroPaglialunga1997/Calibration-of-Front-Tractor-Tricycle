from utility import *
import numpy as np
import math

#--------------------------------------------------------------------------------------------

def compute_laser_odometry(kinematic_parameters, front_trajectory):
    T_off = v2t(np.array([-kinematic_parameters[4], -kinematic_parameters[5],  -kinematic_parameters[6]]))
    T_off_inverse = v2t(np.array([kinematic_parameters[4], kinematic_parameters[5], kinematic_parameters[6]]))
    laser_odometry = []
    for i in range(0, len(front_trajectory)):
        rear_confg = compute_rear_configuration(kinematic_parameters[2], front_trajectory[i, :])
        T_laser = compute_laser_transformation(rear_confg, T_off, T_off_inverse)
        laser_pose = t2v(T_laser)
        laser_odometry.append(laser_pose)
    return np.array(laser_odometry)

#--------------------------------------------------------------------------------------------

def compute_front_wheel_odometry(init_confg, kinematic_parameters, encoders_values, max_enc_values):
    front_odometry = []
    front_odometry.append(init_confg)
    curr_front_confg = init_confg
    for i in range(1, len(encoders_values)):
        delta_inc_enc = encoders_values[i, 1] - encoders_values[i-1, 1]
        delta_confg = prediction(curr_front_confg, kinematic_parameters, delta_inc_enc, encoders_values[i-1:i+1, 0], max_enc_values)
        next_front_confg = curr_front_confg + delta_confg
        front_odometry.append(next_front_confg)
        curr_front_confg = next_front_confg
    return np.array(front_odometry)

#--------------------------------------------------------------------------------------------

def compute_rear_configuration(axis_length, front_confg):
    x_rear = front_confg[0] - axis_length*math.cos(front_confg[2])
    y_rear = front_confg[1] - axis_length*math.sin(front_confg[2])
    return np.array([x_rear, y_rear, front_confg[2], front_confg[3]])

#--------------------------------------------------------------------------------------------

def compute_laser_transformation(rear_confg, T_off, T_off_inverse):
    T_R = v2t(rear_confg)
    T_L = np.matmul(T_off, np.matmul(T_R, T_off_inverse))
    return T_L

#--------------------------------------------------------------------------------------------

def prediction(curr_confg, kinematic_parameters, delta_inc_enc, abs_enc_values, max_enc_values):
    theta = curr_confg[2]
    kt, L = [kinematic_parameters[1], kinematic_parameters[2]]
    psi = new_psi_from_abs_enc(abs_enc_values[0], max_enc_values[0], kinematic_parameters)
    next_psi = new_psi_from_abs_enc(abs_enc_values[1], max_enc_values[0], kinematic_parameters)

    v = kt * delta_inc_enc / (max_enc_values[1]-1) # delta_inc_enc / delta_time is omitted
    x_dot = v * math.cos(theta+psi)
    y_dot = v * math.sin(theta+psi)
    theta_dot = (v / L) * math.sin(psi)
    psi_dot = difference_btw_angles(next_psi,psi)

    return np.array([x_dot, y_dot, theta_dot, psi_dot])

#--------------------------------------------------------------------------------------------

def new_psi_from_abs_enc(final_abs_enc, max_ABS_enc_value, kinematic_parameters):
    ks, steer_off = [kinematic_parameters[0], kinematic_parameters[3]]
    if final_abs_enc > max_ABS_enc_value/2:
        return - (ks * (max_ABS_enc_value - final_abs_enc)*math.pi/(max_ABS_enc_value/2)) + steer_off
    else:
        return (ks * final_abs_enc * math.pi / (max_ABS_enc_value/2)) + steer_off

#--------------------------------------------------------------------------------------------
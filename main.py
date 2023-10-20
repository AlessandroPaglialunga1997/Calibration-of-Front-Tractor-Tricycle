from dataset_handler import *
import numpy as np
import math
import matplotlib.pyplot as plt

#-------------------------Create Clean and Consistent Dataset----------------------------

# remove_unnecessary_comments_and_spaces(old_dataset_path = "Datasets/dataset.txt", #original dataset
#                                        new_dataset_path = "Datasets/clened_dataset.txt", #dataset without comments and unnecessary spaces
#                                        comment_symbol = "#") #if a line starts with this symbol then it is a comment
# make_dataset_consistent(old_dataset_path = "Datasets/clened_dataset.txt", #clean dataset
#                         new_dataset_path = "Datasets/consistent_dataset.txt", #consistent dataset path
#                         info_separator = ":", #each vector (info) is preceded by a name and a separator (e.g. time: ':' is the separator)
#                         timestamp_name = "time",
#                         encoders_info_name = "ticks",
#                         first_timestamp = 1668091584.821040869,
#                         first_inc_enc = 4294859756,
#                         overflow_value = 4294967295) # maximum value for incremental encoder

#--------------------------------------------------------------------------------------------

def read_from_consistent_dataset(consistent_dataset_path, info_separator, timestamp_name, encoders_values_name, robot_confg_name, laser_confg_name):
    timestamp = []
    encoders_values = []
    robot_confg = []
    laser_confg = []
    consistent_dataset_file = open(consistent_dataset_path)
    all_lines = consistent_dataset_file.read().splitlines()
    num_records = len(all_lines)
    for line in all_lines:
        all_tokens = line.split(" ")
        index_token = 0
        for token in all_tokens:
            if token.lower() == timestamp_name + info_separator:
                timestamp.append(float(all_tokens[index_token + 1]))
            elif token.lower() == encoders_values_name + info_separator:
                abs_enc_value = int(all_tokens[index_token + 1])
                inc_enc_value = int(all_tokens[index_token + 2])
                curr_encoders_value_np = np.array([abs_enc_value, inc_enc_value])
                encoders_values.append(curr_encoders_value_np)
            elif token.lower() == robot_confg_name + info_separator:
                x = float(all_tokens[index_token + 1])
                y = float(all_tokens[index_token + 2])
                theta = float(all_tokens[index_token + 3])
                curr_robot_confg_np = np.array([x, y, theta])
                robot_confg.append(curr_robot_confg_np)
            elif token.lower() == laser_confg_name + info_separator:
                x = float(all_tokens[index_token + 1])
                y = float(all_tokens[index_token + 2])
                theta = float(all_tokens[index_token + 3])
                curr_laser_confg_np = np.array([x, y, theta])
                laser_confg.append(curr_laser_confg_np)
            index_token += 1
    return [num_records, np.array(timestamp), np.array(encoders_values), np.array(robot_confg), np.array(laser_confg)]

#--------------------------------------------------------------------------------------------

def dimensions_sanity_checks(num_records, num_encoders, dim_robot_confg_space, dim_laser_confg_space, timestamp, encoders_values, robot_confg, laser_confg):
    assert timestamp.shape[0] == num_records
    assert encoders_values.shape[0] == num_records
    assert encoders_values.shape[1] == num_encoders
    assert robot_confg.shape[0] == num_records
    assert robot_confg.shape[1] == dim_robot_confg_space
    assert laser_confg.shape[0] == num_records
    assert laser_confg.shape[1] == dim_laser_confg_space

#--------------------------------------------------------------------------------------------

# def compute_laser_trajectory(init_confg, kinematic_paramter, encoders_values, max_enc_values):
#     axis_length = kinematic_paramter[2]
#     init_rear_confg = compute_rear_configuration(axis_length, init_confg)
#     rear_trajectory = []
#     rear_trajectory.append(np.array([init_rear_confg[0], init_rear_confg[1], init_rear_confg[2], init_rear_confg[3]])) 
#     curr_front_confg = init_confg
#     for i in range(1, len(encoders_values)):
#         delta_inc_enc = encoders_values[i, 1] - encoders_values[i-1, 1]
#         abs_enc_value = encoders_values[i, 0]
#         delta_front_confg = delta_front_confg_prediction(curr_front_confg, kinematic_paramter, abs_enc_value, delta_inc_enc, max_enc_values)
#         next_confg = curr_front_confg + delta_front_confg # (Note-1) '* delta_time' is omitted because the function called should have a '/ delta_time' 
        
#         next_rear_confg = compute_rear_configuration(axis_length, next_confg)
#         T_R = v2t(next_rear_confg)
#         T_L = np.matmul(T_off, np.matmul(T_R, T_off_inverse))
#         next_laser_confg = t2v(T_L)
#         x_laser = next_laser_confg[0,0] * math.cos(-0.00108296) - next_laser_confg[0,1] * math.sin(-0.00108296)
#         y_laser = next_laser_confg[0,0] * math.sin(-0.00108296) + next_laser_confg[0,1] * math.cos(-0.00108296)
#         rear_trajectory.append(np.array([x_laser, y_laser, 0, 0]))
#         curr_front_confg = next_confg
#     return np.array(rear_trajectory)

#--------------------------------------------------------------------------------------------

def compute_laser_trajectory(init_confg, kinematic_paramter, encoders_values, max_enc_values):
    axis_length = kinematic_paramter[2]
    init_rear_confg = compute_rear_configuration(axis_length, init_confg)
    init_laser_confg = compute_laser_configuration(init_rear_confg)
    laser_trajectory = []
    laser_trajectory.append(init_laser_confg) 
    curr_front_confg = init_confg
    for i in range(1, len(encoders_values)):
        delta_inc_enc = encoders_values[i, 1] - encoders_values[i-1, 1]
        delta_confg = delta_confg_prediction(curr_front_confg, kinematic_paramter, delta_inc_enc, encoders_values[i-1:i+1, 0], max_enc_values)
        next_front_confg = curr_front_confg + delta_confg # '* delta_time' is omitted
        
        next_rear_confg = compute_rear_configuration(axis_length, next_front_confg)
        next_laser_confg = compute_laser_configuration(next_rear_confg)
        laser_trajectory.append(next_laser_confg)
        curr_front_confg = next_front_confg
    return np.array(laser_trajectory)

#--------------------------------------------------------------------------------------------

def compute_rear_configuration(axis_length, front_confg):
    x_rear = front_confg[0] - axis_length*math.cos(front_confg[2])
    y_rear = front_confg[1] - axis_length*math.sin(front_confg[2])
    return np.array([x_rear, y_rear, front_confg[2], front_confg[3]])

#--------------------------------------------------------------------------------------------

def compute_laser_configuration(rear_confg):
    T_R = v2t(rear_confg)
    T_L = np.matmul(T_off, np.matmul(T_R, T_off_inverse))
    next_laser_confg = t2v(T_L)
    laser_orient_wrt_robot = v2t([0, 0, laser_rotation_wrt_robot[2]])
    x_laser = next_laser_confg[0] * math.cos(-0.00108296) - next_laser_confg[1] * math.sin(-0.00108296)
    y_laser = next_laser_confg[0] * math.sin(-0.00108296) + next_laser_confg[1] * math.cos(-0.00108296)
    return np.array([x_laser, y_laser])

#--------------------------------------------------------------------------------------------

def delta_confg_prediction(curr_confg, kinematic_paramter, delta_inc_enc, abs_enc_values,max_enc_values):
    theta, psi = [curr_confg[2], curr_confg[3]]
    kt, L = [kinematic_paramter[1], kinematic_paramter[2]]
    
    v = kt * delta_inc_enc / (max_enc_values[1]-1) # delta_inc_enc / delta_time is omitted
    x_dot = v * math.cos(theta+psi)
    y_dot = v * math.sin(theta+psi)
    theta_dot = (v / L) * math.sin(psi)
    curr_psi = new_psi_from_abs_enc(abs_enc_values[0], max_enc_values[0], kinematic_paramter)
    next_psi = new_psi_from_abs_enc(abs_enc_values[1], max_enc_values[0], kinematic_paramter) 
    psi_dot = next_psi - curr_psi

    return np.array([x_dot, y_dot, theta_dot, psi_dot])

#--------------------------------------------------------------------------------------------

def new_psi_from_abs_enc(final_abs_enc, max_ABS_enc_value, kinematic_paramter):  
    ks, steer_off = [kinematic_paramter[0], kinematic_paramter[3]]
    if final_abs_enc > max_ABS_enc_value/2:
        return - (ks * (max_ABS_enc_value - final_abs_enc)*math.pi/(max_ABS_enc_value/2)) + steer_off
    else:
        return (ks * final_abs_enc * math.pi / (max_ABS_enc_value/2)) + steer_off
#--------------------------------------------------------------------------------------------

def t2v(transformation):
    vector = np.zeros((1,3))
    vector[0, 0:2] = transformation[0:2,2]
    vector[0, 2] = math.atan2(transformation[1,0], transformation[0,0])
    return np.array(vector[0, :])

#--------------------------------------------------------------------------------------------

def v2t(vector):
    cos = math.cos(vector[2])
    sin = math.sin(vector[2])
    transformation = np.array([[cos, -sin, vector[0]],[sin, cos, vector[1]],[0, 0, 1]])
    return np.array(transformation)

#--------------------------------------------------------------------------------------------

info_separator = ":"
timestamp_name = "time"
encoders_values_name = "ticks"
num_encoders = 2
max_enc_values = [8192, 5000] #[max_INC_enc_value, max_ABS_enc_value]
robot_confg_name = "model_pose"
dim_robot_confg_space = 3
laser_confg_name = "tracker_pose"
dim_laser_confg_space = 3
consistent_dataset_path = "Datasets/consistent_dataset.txt"

[num_records, timestamp, encoders_values, robot_odometry_with_initial_guess, laser_odometry] = read_from_consistent_dataset(consistent_dataset_path, info_separator, timestamp_name, encoders_values_name, robot_confg_name, laser_confg_name)

dimensions_sanity_checks(num_records, num_encoders, dim_robot_confg_space, dim_laser_confg_space, timestamp, encoders_values, robot_odometry_with_initial_guess, laser_odometry)

#kinematic_paramter = np.array([0.564107, 0.0106141, 1.54757, -0.0559079]) #[Ks, Kt, axis_length, steer_off]
kinematic_paramter = np.array([0.1, 0.0106141, 1.4, 0]) #[Ks, Kt, axis_length, steer_off]

initial_front_confg = np.array([1.54757, 0, 0, new_psi_from_abs_enc(encoders_values[0, 0], max_enc_values[0], kinematic_paramter)]) #[x, y, theta, phi]

laser_pos_wrt_robot = np.array([1.81022, -0.0228018, 0])
laser_rotation_wrt_robot =  np.array([0, 0, -0.00108296, 0.999999])
from_robot_to_laser = v2t(np.array([laser_pos_wrt_robot[0], laser_pos_wrt_robot[1],  laser_rotation_wrt_robot[2]]))

T_off = v2t(np.array([-laser_pos_wrt_robot[0], -laser_pos_wrt_robot[1],  -laser_rotation_wrt_robot[2]]))
T_off_inverse = v2t(np.array([laser_pos_wrt_robot[0], laser_pos_wrt_robot[1], laser_rotation_wrt_robot[2]]))

predicted_laser_odometry = compute_laser_trajectory(initial_front_confg, kinematic_paramter, encoders_values, max_enc_values)

num_points = -1
plt.plot(laser_odometry[0:num_points, 0], laser_odometry[0:num_points, 1])
plt.plot(predicted_laser_odometry[0:num_points, 0], predicted_laser_odometry[0:num_points, 1])
plt.show()


# import math
# def prediction(current_model_pose, parameters, IncEnc, AbsEnc):
#     Ks, Kt, axis_length, steer_off = [parameters[0], parameters[1], parameters[2], parameters[3]]
#     DeltaIncEnc = IncEnc[1] - IncEnc[0]
#     curr_x = current_model_pose[0]
#     curr_y = current_model_pose[1]
#     curr_theta = current_model_pose[2]
#     curr_phi = Ks*AbsEnc[0] - steer_off*math.pi/180
#     cos_theta_phi = math.cos(curr_theta + curr_phi)
#     sin_theta_phi = math.sin(curr_theta + curr_phi)
#     sin_phi = math.sin(curr_phi)
#     new_x = curr_x + cos_theta_phi*Kt*DeltaIncEnc
#     new_y = curr_y + sin_theta_phi*Kt*DeltaIncEnc
#     new_theta = curr_theta + sin_phi*Kt*DeltaIncEnc/axis_length
#     return [new_x, new_y, new_theta]

# def front_rear_position(x, y, theta, axis_length, switch):
#     new_x = x + switch*axis_length*math.cos(theta)
#     new_y = y + switch*axis_length*math.sin(theta)
#     return [new_x, new_y, theta]

# # compute trajectory given parameter values and encoders informations
# def compute_trajectory(parameters, IncEnc_array, AbsEnc_array):
#     Ks, Kt, axis_length, steer_off = [parameters[0], parameters[1], parameters[2], parameters[3]]
#     num_timestamp = len(IncEnc_array)
#     model_pose = []
#     current_model_pose = [0, 0, 0]
#     for i in range(num_timestamp-1):
#         IncEnc = [IncEnc_array[i], IncEnc_array[i+1]]
#         AbsEnc = [AbsEnc_array[i], AbsEnc_array[i+1]]
#         curr_front_model_pose = front_rear_position(current_model_pose[0], current_model_pose[1], current_model_pose[2], axis_length, 1)
#         new_front_model_pose = prediction(curr_front_model_pose, parameters, IncEnc, AbsEnc)
#         new_model_pose = front_rear_position(new_front_model_pose[0], new_front_model_pose[1], new_front_model_pose[2], axis_length, -1)
#         model_pose.append(new_model_pose)
#         current_model_pose = new_model_pose
#     return np.asarray(model_pose)

# def error(state, measurement, solution, IncEnc, AbsEnc):
#     new_front_model_pose = prediction(state, solution, IncEnc, AbsEnc)
#     pose_prediction = front_rear_position(new_front_model_pose[0], new_front_model_pose[1], new_front_model_pose[2], solution[2], -1)
#     return [pose_prediction, pose_prediction - measurement]

# def Jacobian(state, solution, IncEnc, AbsEnc):
#     J = np.zeros((3, 4))
#     dx = np.zeros(4)
#     epsilon = 1e-4
#     for i in range(len(dx)):
#         dx[i] = epsilon
#         first_prediction = np.array(prediction(state, solution + dx, IncEnc, AbsEnc))
#         second_prediction = np.array(prediction(state, solution - dx, IncEnc, AbsEnc))
#         J[:, i] = first_prediction - second_prediction
#         dx[i] = 0
#     J *= 0.5/epsilon
#     return J

# from numpy import linalg
# def ls_calibrate_odometry(initial_state, measurements, initial_solution, IncEnc_array, AbsEnc_array):
#     H = np.zeros((4, 4))
#     b = np.zeros((4, 1))
#     dx = np.zeros((4, 1))
#     front_initial_state = front_rear_position(initial_state[0], initial_state[1], initial_state[2], initial_solution[2], 1)
#     for i in range(len(measurements) - 1):
#         IncEnc = [IncEnc_array[i], IncEnc_array[i+1]]
#         AbsEnc = [AbsEnc_array[i], AbsEnc_array[i+1]]
#         z = measurements[i+1, :]
#         pose_prediction, e = error(front_initial_state, z, initial_solution, IncEnc, AbsEnc)
#         e = np.reshape(e, (3, 1))
#         J = Jacobian(front_initial_state, initial_solution, IncEnc, AbsEnc)
#         H += np.matmul(np.transpose(J), J)
#         b += np.matmul(np.transpose(J), e)
#         front_initial_state = front_rear_position(pose_prediction[0], pose_prediction[1], pose_prediction[2], initial_solution[2], 1)
#     dx = -np.matmul(linalg.pinv(H), b)
#     return initial_solution + np.transpose(dx)



# import numpy as np
# data = np.genfromtxt('dataset/consistent_dataset.txt',
#                      dtype = float,
#                      delimiter=' ')
# time = data[:, 0]
# ticks = data[:, 1:3]
# model_pose = data[:, 3:6]
# tracker_pose = data[:, 6:9]
# tracker_pose = data[:, 6:9]
# # Ground Truth trajectory
# # plot_trajectory(model_pose[:, 0], model_pose[:, 1])

# #                           [Ksteer, Ktraction, axis_length, steer_offset ]
# # [0.1,    0.0106141, 1.4,         0]
# initial_solution = np.array([math.pi*0.1/8192, 5, -0.5, -20])
# AbsEnc_array = ticks[:, 0]
# IncEnc_array = ticks[:, 1]

# # Uncalibrated Odometry
# # uncalibrated_model_pose = compute_trajectory(initial_solution, IncEnc_array, AbsEnc_array)
# # plot_trajectory(uncalibrated_model_pose[:, 0], uncalibrated_model_pose[:, 1])

# # Calibration Parameters
# calibrated_parameters = initial_solution
# for i in range(5):
#     print(i)
#     calibrated_parameters = ls_calibrate_odometry(model_pose[0, :], model_pose, calibrated_parameters, IncEnc_array, AbsEnc_array)
#     calibrated_parameters = np.reshape(calibrated_parameters, (4))
# print(calibrated_parameters)
# calibrated_model_pose = compute_trajectory(calibrated_parameters, IncEnc_array, AbsEnc_array)
# # plot_trajectory(calibrated_model_pose[:, 0], calibrated_model_pose[:, 1])

# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.scatter(calibrated_model_pose[0:100, 0], calibrated_model_pose[0:100, 1])
# ax.scatter(model_pose[0:100, 0], model_pose[0:100, 1])
# # for i in range(0, len(x)):
# # 	ax.annotate(i, (x[i], y[i]))
# ax.axis("equal")
# plt.show()
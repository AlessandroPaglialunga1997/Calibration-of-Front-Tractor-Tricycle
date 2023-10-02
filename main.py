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

def compute_robot_trajectory(initial_state, kinematic_paramter, encoders_values, max_enc_values):
    trajectory = []
    trajectory.append(initial_state)
    curr_state = initial_state
    for i in range(1, len(encoders_values)):
        delta_inc_enc = encoders_values[i, 1] - encoders_values[i-1, 1]
        new_abs_enc = encoders_values[i, 0]
        next_state = next_robot_confg_prediction(kinematic_paramter, curr_state, delta_inc_enc, new_abs_enc, max_enc_values)
        trajectory.append(next_state)
        curr_state = next_state
    return np.array(trajectory)

#--------------------------------------------------------------------------------------------

def next_robot_confg_prediction(kinematic_paramter, curr_state, delta_inc_enc, new_abs_enc, max_enc_values):
    Ks, Kt, axis_length, steer_off = [kinematic_paramter[0], kinematic_paramter[1], kinematic_paramter[2], kinematic_paramter[3]]
    curr_x, curr_y, curr_theta, curr_phi = [curr_state[0], curr_state[1], curr_state[2], curr_state[3]]
    max_ABS_enc_value, max_INC_enc_value = [max_enc_values[0], max_enc_values[1]]
    sin_phi = math.sin(curr_phi)
    cos_theta = math.cos(curr_theta)
    sin_theta = math.sin(curr_theta)
    new_x = curr_x + cos_theta*Kt*delta_inc_enc/max_INC_enc_value 
    new_y = curr_y + sin_theta*Kt*delta_inc_enc/max_INC_enc_value
    new_theta = wrap_angles(curr_theta + sin_phi*Kt*delta_inc_enc/(max_INC_enc_value*axis_length))
    if new_abs_enc < max_ABS_enc_value/2:
        new_phi = (new_abs_enc*Ks)/(max_ABS_enc_value/(2*math.pi)) - steer_off
    else:
        new_phi = -(max_ABS_enc_value-new_abs_enc)*Ks/(max_ABS_enc_value/(2*math.pi)) + steer_off
    new_phi = wrap_angles(new_phi)
    return np.array([new_x, new_y, new_theta, new_phi])

#--------------------------------------------------------------------------------------------

def wrap_angles(angle):
    if (angle > 0 and angle < math.pi) or (angle < 0 and angle > -math.pi):
        return angle
    else:
        cycles = int(angle/math.pi)
        return angle - cycles*math.pi

#--------------------------------------------------------------------------------------------

def t2v(transformation):
    vector = np.zeros((1,3))
    vector[0, 0:2] = transformation[0:2,2]
    vector[0, 2] = math.atan2(transformation[1,0], transformation[0,0])
    return vector

#--------------------------------------------------------------------------------------------

def v2t(vector):
    cos = math.cos(vector[2])
    sin = math.sin(vector[2])
    transformation = np.array([[cos, -sin, vector[0]],[sin, cos, vector[1]],[0, 0, 1]])
    return transformation

#--------------------------------------------------------------------------------------------

def compute_robot_odometry(laser_odometry, T_off, T_off_inverse):
    robot_odometry = []
    for i in range(len(laser_odometry)):
        T_L = v2t(laser_odometry[i, :])
        T_R = np.matmul(T_off_inverse, np.matmul(T_L, T_off))
        odom_R = t2v(T_R)
        robot_odometry.append(odom_R)
    robot_odometry = np.array(robot_odometry)
    robot_odometry = np.squeeze(robot_odometry, axis=1)
    return robot_odometry

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

kinematic_paramter = np.array([0.564107, 0.0106141, 1.54757, -0.0559079]) #[Ks, Kt, axis_length, steer_off]
#kinematic_paramter = np.array([0.1, 0.0106141, 1.4, 0]) #[Ks, Kt, axis_length, steer_off]

initial_robot_confg = np.array([0, 0, 0, 0]) #[x, y, theta, phi]

laser_pos_wrt_robot = np.array([1.81022, -0.0228018, 0])
laser_rotation_wrt_robot =  np.array([0, 0, -0.00108296, 0.999999])

T_off = v2t(np.array([laser_pos_wrt_robot[0], laser_pos_wrt_robot[1],  laser_rotation_wrt_robot[2]]))
T_off_inverse = v2t(np.array([-laser_pos_wrt_robot[0], -laser_pos_wrt_robot[1], -laser_rotation_wrt_robot[2]]))

true_robot_odometry = compute_robot_odometry(laser_odometry, T_off, T_off_inverse)

robot_odometry = compute_robot_odometry(laser_odometry, T_off, T_off_inverse)
predicted_robot_odometry = compute_robot_trajectory(initial_robot_confg, kinematic_paramter, encoders_values, max_enc_values)

num_points = -1
#plt.plot(laser_odometry[0:num_points, 0], laser_odometry[0:num_points, 1])
#plt.plot(robot_odometry_with_initial_guess[0:num_points, 0], robot_odometry_with_initial_guess[0:num_points, 1])
plt.plot(predicted_robot_odometry[0:num_points, 0], predicted_robot_odometry[0:num_points, 1])
plt.plot(true_robot_odometry[0:num_points, 0], true_robot_odometry[0:num_points, 1])
plt.show()

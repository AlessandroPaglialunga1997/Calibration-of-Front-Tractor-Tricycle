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

def compute_laser_trajectory(initial_confg, kinematic_paramter, encoders_values, max_enc_values):
    #plotta traiettoria centro cinematico
    #dalla traiattoria trova il laser T_off*T_R*T_off_inverse
    trajectory = []
    # angle = initial_confg[2]
    # rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    # vector = [laser_pos_wrt_robot[0]-kinematic_paramter[2], laser_pos_wrt_robot[1]]
    # rotated_vector = np.dot(rotation_matrix, vector)
    # a = initial_confg + np.array([rotated_vector[0], rotated_vector[1], 0, 0])
    # T_R = v2t([initial_confg[0], initial_confg[1], initial_confg[2]]) 
    # b = np.matmul(T_off, T_R)
    # c = t2v(b)
    # trajectory.append(np.array([c[0,0], c[0,1], 0, 0])) 
    trajectory.append(np.array([initial_confg[0]-kinematic_paramter[2], initial_confg[1], initial_confg[2], initial_confg[3]])) 
    curr_confg = initial_confg
    for i in range(1, len(encoders_values)):
        delta_inc_enc = encoders_values[i, 1] - encoders_values[i-1, 1]
        delta_confg = delta_confg_prediction(curr_confg, kinematic_paramter, delta_inc_enc, max_enc_values[1])
        next_confg = curr_confg + delta_confg # delta_confg * delta_time is omitted
        new_psi = new_psi_from_abs_enc(encoders_values[i, 0], max_enc_values[0], kinematic_paramter)
        next_confg[3] = new_psi
        
        # new_theta = next_confg[2]
        # rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
        # laser_offset = [laser_pos_wrt_robot[0]-kinematic_paramter[2], laser_pos_wrt_robot[1]]
        # rotated_laser_offset = np.dot(rotation_matrix, laser_offset)

        # curr_laser_pos_wrt_robot = next_confg + np.array([rotated_laser_offset[0], rotated_laser_offset[1], 0, 0])
        # T_R = v2t([next_confg[0], next_confg[1], angle]) 
        # b = np.matmul(T_off, T_R)
        # c = t2v(b)
        # trajectory.append(np.array([c[0,0], c[0,1], 0, 0]))
        x_rear = next_confg[0] - kinematic_paramter[2]*math.cos(next_confg[2])
        y_rear = next_confg[1] - kinematic_paramter[2]*math.sin(next_confg[2])
        T_R = v2t([x_rear, y_rear, next_confg[2]])
        T_L = np.matmul(T_off, np.matmul(T_R, T_off_inverse))
        spero = t2v(T_L)
        x_laser = spero[0,0] * math.cos(-0.00108296) - spero[0,1] * math.sin(-0.00108296)
        y_laser = spero[0,0] * math.sin(-0.00108296) + spero[0,1] * math.cos(-0.00108296)
        trajectory.append(np.array([x_laser, y_laser, 0, 0]))
        curr_confg = next_confg
    return np.array(trajectory)

#--------------------------------------------------------------------------------------------

def delta_confg_prediction(curr_confg, kinematic_paramter, delta_inc_enc, max_INC_enc_value):
    theta, psi = [curr_confg[2], curr_confg[3]]
    kt, L = [kinematic_paramter[1], kinematic_paramter[2]]
    
    v = kt * delta_inc_enc / (max_INC_enc_value-1) # delta_inc_enc / delta_time is omitted

    x_dot = v * math.cos(theta+psi)

    y_dot = v * math.sin(theta+psi)

    theta_dot = (v / L) * math.sin(psi)

    return np.array([x_dot, y_dot, theta_dot, 0])

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
    return vector

#--------------------------------------------------------------------------------------------

def v2t(vector):
    cos = math.cos(vector[2])
    sin = math.sin(vector[2])
    transformation = np.array([[cos, -sin, vector[0]],[sin, cos, vector[1]],[0, 0, 1]])
    return transformation

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

initial_robot_confg = np.array([1.54757, 0, 0, 0]) #[x, y, theta, phi]

laser_pos_wrt_robot = np.array([1.81022, -0.0228018, 0])
laser_rotation_wrt_robot =  np.array([0, 0, -0.00108296, 0.999999]) ## ADD ##
from_robot_to_laser = v2t(np.array([laser_pos_wrt_robot[0], laser_pos_wrt_robot[1],  laser_rotation_wrt_robot[2]]))

from_laser_to_robot = np.linalg.inv(from_robot_to_laser)
robot_pos_wrt_laser = np.array([-1.81022, 0.0228018, 0])
robot_rotation_wrt_laser =  np.array([0, 0, -0.00108296, 0.999999]) ## ADD ##

T_off = v2t(np.array([robot_pos_wrt_laser[0], robot_pos_wrt_laser[1],  robot_rotation_wrt_laser[2]]))
T_off_inverse = v2t(np.array([-robot_pos_wrt_laser[0], -robot_pos_wrt_laser[1], -robot_rotation_wrt_laser[2]]))

predicted_laser_odometry = compute_laser_trajectory(initial_robot_confg, kinematic_paramter, encoders_values, max_enc_values)

num_points = -1
plt.plot(laser_odometry[0:num_points, 0], laser_odometry[0:num_points, 1])
plt.plot(predicted_laser_odometry[0:num_points, 0], predicted_laser_odometry[0:num_points, 1])
plt.show()
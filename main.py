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

def read_from_consistent_dataset(consistent_dataset_path, info_separator, timestamp_name, encoders_values_name, robot_confg_name, sensor_confg_name):
    timestamp = []
    encoders_values = []
    robot_confg = []
    sensor_confg = []
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
            elif token.lower() == sensor_confg_name + info_separator:
                x = float(all_tokens[index_token + 1])
                y = float(all_tokens[index_token + 2])
                theta = float(all_tokens[index_token + 3])
                curr_sensor_confg_np = np.array([x, y, theta])
                sensor_confg.append(curr_sensor_confg_np)
            index_token += 1
    return [num_records, np.array(timestamp), np.array(encoders_values), np.array(robot_confg), np.array(sensor_confg)]

#--------------------------------------------------------------------------------------------

def dimensions_sanity_checks(num_records, num_encoders, dim_robot_confg_space, dim_sensor_confg_space, timestamp, encoders_values, robot_confg, sensor_confg):
    assert timestamp.shape[0] == num_records
    assert encoders_values.shape[0] == num_records
    assert encoders_values.shape[1] == num_encoders
    assert robot_confg.shape[0] == num_records
    assert robot_confg.shape[1] == dim_robot_confg_space
    assert sensor_confg.shape[0] == num_records
    assert sensor_confg.shape[1] == dim_sensor_confg_space

#--------------------------------------------------------------------------------------------
def compute_trajectory(initial_state, parameters, sensor_confg):
    trajectory = []
    trajectory.append(initial_state)
    curr_state = initial_state
    for i in range(1, len(sensor_confg)):
        delta_inc_enc = sensor_confg[i, 1] - sensor_confg[i-1, 1]
        #print([sensor_confg[i, 1], sensor_confg[i-1, 1], delta_inc_enc, i])
        curr_abs_enc = sensor_confg[i, 0]
        next_state = prediction(parameters, curr_state, delta_inc_enc, curr_abs_enc)
        trajectory.append(next_state)
        curr_state = next_state
    return np.array(trajectory)

#--------------------------------------------------------------------------------------------

def prediction(parameters, curr_state, delta_inc_enc, curr_abs_enc):
    Ks, Kt, axis_length, steer_off = [parameters[0], parameters[1], parameters[2], parameters[3]]
    curr_theta, curr_phi = [curr_state[2], curr_state[3]]
    cos_theta_phi = math.cos(curr_theta + curr_phi)
    sin_theta_phi = math.sin(curr_theta + curr_phi)
    sin_phi = math.sin(curr_phi)
    new_x = curr_state[0] + cos_theta_phi*Kt*delta_inc_enc
    new_y = curr_state[1] + sin_theta_phi*Kt*delta_inc_enc
    new_theta = curr_state[2] + sin_phi*Kt*delta_inc_enc/axis_length
    new_phi = math.pi*Ks*curr_abs_enc/180 - math.pi*steer_off/180
    return np.array([new_x, new_y, new_theta, new_phi])

#--------------------------------------------------------------------------------------------

info_separator = ":"
timestamp_name = "time"
encoders_values_name = "ticks"
num_encoders = 2
robot_confg_name = "model_pose"
dim_robot_confg_space = 3
sensor_confg_name = "tracker_pose"
dim_sensor_confg_space = 3
consistent_dataset_path = "Datasets/consistent_dataset.txt"

[num_records, timestamp, encoders_values, robot_confg, sensor_confg] = read_from_consistent_dataset(consistent_dataset_path, info_separator, timestamp_name, encoders_values_name, robot_confg_name, sensor_confg_name)
dimensions_sanity_checks(num_records, num_encoders, dim_robot_confg_space, dim_sensor_confg_space, timestamp, encoders_values, robot_confg, sensor_confg)
initial_guess = np.array([360/8192, 0.0106141, 1.4, 0]) #[Ksteer, Ktraction, axis_length, steer_offset]
initial_x = 0
initial_y = 0
initial_theta = 0
initial_phi = math.pi*initial_guess[0]*sensor_confg[0,0]/180 - math.pi*initial_guess[3]/180
initial_robot_confg = np.array([initial_x, initial_y, initial_theta, initial_phi])
trajectory = compute_trajectory(initial_robot_confg, initial_guess, encoders_values)

plt.plot(trajectory[:, 0], trajectory[:, 1])
plt.plot(robot_confg[:, 0], robot_confg[:, 1])
plt.show()
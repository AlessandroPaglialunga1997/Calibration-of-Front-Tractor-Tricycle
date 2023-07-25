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

def compute_trajectory(initial_state, parameters, encoders_values, robot_confg, phi_trajectory):
    trajectory = []
    trajectory.append(initial_state)
    curr_state = initial_state
    for i in range(1, len(encoders_values)):
        delta_inc_enc = encoders_values[i, 1] - encoders_values[i-1, 1]
        new_abs_enc = encoders_values[i, 0]
        curr_abs_enc = encoders_values[i-1, 0]
        next_state = prediction(parameters, curr_state, delta_inc_enc, new_abs_enc, curr_abs_enc, robot_confg[i, :], i, phi_trajectory)
        trajectory.append(next_state)
        curr_state = next_state
    return np.array(trajectory)

#--------------------------------------------------------------------------------------------

def prediction(parameters, curr_state, delta_inc_enc, new_abs_enc, curr_abs_enc, ith_robot_confg, i, phi_trajectory):
    Ks, Kt, axis_length, steer_off = [parameters[0], parameters[1], parameters[2], parameters[3]]
    curr_x, curr_y, curr_theta, curr_phi = [curr_state[0], curr_state[1], curr_state[2], phi_trajectory[i-1]]
    sin_phi = math.sin(curr_phi)
    cos_theta = math.cos(curr_theta)
    sin_theta = math.sin(curr_theta)
    new_x = curr_x + cos_theta*Kt*delta_inc_enc/5000 
    new_y = curr_y + sin_theta*Kt*delta_inc_enc/5000
    new_theta = wrap_angles(curr_theta + sin_phi*Kt*delta_inc_enc/(5000*axis_length))
    new_phi = 0.0007669*new_abs_enc
    #print(i, delta_inc_enc, new_abs_enc, (new_theta + new_phi - steer_off)*8192/(2*math.pi), new_theta, new_phi)
    return np.array([new_x, new_y, new_theta, new_phi])

#--------------------------------------------------------------------------------------------

def wrap_abs_enc(abs_enc, max_value):
    if abs_enc < max_value/2:
        return math.pi*abs_enc/(max_value/2)/4
    elif abs_enc > max_value/2:
        return -math.pi*(max_value-abs_enc)/(max_value/2)/4
    else:
        return math.pi

#--------------------------------------------------------------------------------------------

def wrap_angles(angle): 
    new_angle = 0
    if angle > 0 and angle < math.pi:
        return angle
    elif angle < 0 and angle > -math.pi:
        return angle
    elif angle > 0 and angle > math.pi:
        cycles = int(angle/(2*math.pi))
        new_angle = angle - cycles*2*math.pi #btn 0 and 2pi
        if new_angle < math.pi:
            return new_angle
        else:
            return new_angle - 2*math.pi
    elif angle < 0 and angle < -math.pi:
        cycles = int(angle/(2*math.pi))
        new_angle = angle - cycles*2*math.pi #btn 0 and 2pi
        if new_angle > -math.pi:
            return new_angle
        else:
            return new_angle + 2*math.pi
    else:
        return 0
    
#--------------------------------------------------------------------------------------------

def compute_rear_trajectory(front_trajectory, axis_length):
    num_points = len(front_trajectory)
    all_x = np.reshape(front_trajectory[:, 0] - axis_length * np.cos(front_trajectory[:, 2]), (num_points,1))
    all_y = np.reshape(front_trajectory[:, 1] - axis_length * np.sin(front_trajectory[:, 2]), (num_points,1))
    rear_trajectory = np.concatenate((all_x, all_y), axis=1)
    return rear_trajectory

#--------------------------------------------------------------------------------------------

def compute_new_phi(curr_theta, new_theta, delta_inc_enc, axis_length, Kt, curr_phi):
    if delta_inc_enc == 0:
        return curr_phi
    sin_phi = (new_theta - curr_theta)*5000*axis_length/(Kt*delta_inc_enc)
    return math.asin(sin_phi)

#--------------------------------------------------------------------------------------------

def compute_phi_trajectory(theta_trajectory, inc_enc, axis_length, Kt, initial_phi):
    phi_trajectory = []
    phi_trajectory.append(initial_phi)
    curr_phi = initial_phi
    for i in range(1, len(encoders_values)):
        curr_theta = theta_trajectory[i-1]
        new_theta = theta_trajectory[i]
        delta_inc_enc = inc_enc[i] - inc_enc[i-1]
        new_phi = compute_new_phi(curr_theta, new_theta, delta_inc_enc, axis_length, Kt, curr_phi)
        phi_trajectory.append(new_phi)
        curr_phi = new_phi
    return np.array(phi_trajectory)

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

# ks*abs_enc/
dimensions_sanity_checks(num_records, num_encoders, dim_robot_confg_space, dim_sensor_confg_space, timestamp, encoders_values, robot_confg, sensor_confg)
Ks = 0.564107
Kt = 0.0106141
axis_length = 1.54757
steer_off = -0.0559079
initial_guess = np.array([Ks, Kt, axis_length, steer_off]) #2*290*180/8192 #0.564107
initial_x = 0
initial_y = 0 #72,892568723
initial_theta = 0
initial_phi = 0.024587744106089455
initial_robot_confg = np.array([initial_x, initial_y, initial_theta, initial_phi])
phi_trajectory = compute_phi_trajectory(robot_confg[:, 2], encoders_values[:, 1], axis_length, Kt, initial_phi)
rear_trajectory = compute_trajectory(initial_robot_confg, initial_guess, encoders_values, robot_confg, phi_trajectory)
phi_trajectory[0] = -1.0

num_point = -1
plt.plot(rear_trajectory[:num_point, 0], rear_trajectory[:num_point, 1])
plt.plot(robot_confg[:num_point, 0], robot_confg[:num_point, 1])







# (theta + steer)*8192/(2*pi*ks)
plt.show()
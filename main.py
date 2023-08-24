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
        new_phi = (new_abs_enc*Ks)/(max_ABS_enc_value-1) - steer_off
    else:
        new_phi = -(max_ABS_enc_value-new_abs_enc)*Ks/(max_ABS_enc_value-1) + steer_off
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

def compute_sensor_trajectory(laser_translation_wrt_baselink, robot_trajectory):
    trajectory = []
    for i in range(0, len(robot_trajectory)):
        theta = robot_trajectory[i, 2]
        sin = math.sin(theta)
        cos = math.cos(theta)
        Rz = np.array([[cos, -sin], [sin, cos]])
        a = np.reshape(laser_translation_wrt_baselink[0:2], (2,1))
        b = np.reshape(np.matmul(Rz, a), (1,2))
        trajectory.append(robot_trajectory[i, 0:2] + b)
    trajectory = np.reshape(trajectory, (2434, 2))
    return trajectory

#--------------------------------------------------------------------------------------------

def sensor_trajectory_by_odom(laser_translation_wrt_baselink, robot_trajectory, sensor_odometry):
    trajectory_by_odometry = []
    trajectory_by_odometry.append(laser_translation_wrt_baselink[0:2].tolist())
    for i in range(1, len(robot_trajectory)):
        sensor_theta = sensor_odometry[i, 2]
        sin = math.sin(sensor_theta)
        cos = math.cos(sensor_theta)
        Rz = np.array([[cos, -sin], [sin, cos]])
        a = np.reshape(sensor_odometry[i, 0:2] - sensor_odometry[i-1, 0:2], (2,1))
        b = np.reshape(np.matmul(Rz, a), (1,2))
        c = np.reshape(robot_trajectory[i, 0:2] + laser_translation_wrt_baselink[0:2] + b, (2))
        trajectory_by_odometry.append(c.tolist())
    trajectory_by_odometry = np.array(trajectory_by_odometry)
    return trajectory_by_odometry

#--------------------------------------------------------------------------------------------

def prova(laser_translation_wrt_baselink, robot_trajectory, sensor_odometry):
    trajectory_by_odometry = []
    trajectory_by_odometry.append(laser_translation_wrt_baselink[0:2].tolist())
    for i in range(1, len(robot_trajectory)):
        b = np.reshape(sensor_odometry[i, 0:2] - sensor_odometry[i-1, 0:2], (1,2))
        c = np.reshape(robot_trajectory[i, 0:2] + laser_translation_wrt_baselink[0:2] + b, (2))
        trajectory_by_odometry.append(c.tolist())
    trajectory_by_odometry = np.array(trajectory_by_odometry)
    return trajectory_by_odometry

#--------------------------------------------------------------------------------------------

info_separator = ":"
timestamp_name = "time"
encoders_values_name = "ticks"
num_encoders = 2
max_enc_values = [8192, 5000] #[max_INC_enc_value, max_ABS_enc_value]
robot_confg_name = "model_pose"
dim_robot_confg_space = 3
sensor_confg_name = "tracker_pose"
dim_sensor_confg_space = 3
consistent_dataset_path = "Datasets/consistent_dataset.txt"

[num_records, timestamp, encoders_values, robot_trajectory, sensor_odometry] = read_from_consistent_dataset(consistent_dataset_path, info_separator, timestamp_name, encoders_values_name, robot_confg_name, sensor_confg_name)

dimensions_sanity_checks(num_records, num_encoders, dim_robot_confg_space, dim_sensor_confg_space, timestamp, encoders_values, robot_trajectory, sensor_odometry)
kinematic_paramter = np.array([0.564107, 0.0106141, 1.54757, -0.0559079]) #[Ks, Kt, axis_length, steer_off]
initial_robot_confg = np.array([0, 0, 0, 0]) #[x, y, theta, phi]

# rear_trajectory = compute_robot_trajectory(initial_robot_confg, kinematic_paramter, encoders_values, max_enc_values)
# num_point = -1
# plt.plot(rear_trajectory[:num_point, 0], rear_trajectory[:num_point, 1])
# plt.plot(robot_trajectory[:num_point, 0], robot_trajectory[:num_point, 1])
# plt.show()

laser_translation_wrt_baselink = np.array([ 1.81022, -0.0228018, 0 ])
laser_rotation_wrt_baselink = np.array([ 0, 0, -0.00108296, 0.999999 ])
initial_sensor_confg = np.array([0, 0, 0, 0])
sensor_trajectory = compute_sensor_trajectory(laser_translation_wrt_baselink, robot_trajectory)

num_point = -1
plt.plot(sensor_trajectory[:num_point, 0], sensor_trajectory[:num_point, 1])
plt.plot(robot_trajectory[:num_point, 0], robot_trajectory[:num_point, 1])
plt.show()
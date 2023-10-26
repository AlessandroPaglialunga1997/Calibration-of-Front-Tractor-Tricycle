from dataset_handler import *
from utility import *
from front_tractor_tricycle import *
from ls_odometry_calibration import *
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

# initializes the main system variables
info_separator = ":"
timestamp_name = "time"
encoders_values_name = "ticks"
num_encoders = 2
max_enc_values = [8192, 5000] #[max_INC_enc_value, max_ABS_enc_value]
robot_pose_name = "model_pose"
dim_robot_pose_space = 3
laser_pose_name = "tracker_pose"
dim_laser_pose_space = 3
consistent_dataset_path = "Datasets/consistent_dataset.txt"

# read from clened and consistent dataset
[num_records, timestamp, encoders_values, robot_odometry_with_initial_guess, laser_odometry] = read_from_consistent_dataset(consistent_dataset_path, info_separator, timestamp_name, encoders_values_name, robot_pose_name, laser_pose_name)

# check that all dimensions are consistent
dimensions_sanity_checks(num_records, num_encoders, dim_robot_pose_space, dim_laser_pose_space, timestamp, encoders_values, robot_odometry_with_initial_guess, laser_odometry)

# initialize the kinematic parameters
kinematic_parameters = np.array([0.564107, 0.0106141, 1.54757, -0.0559079]) #[Ks, Kt, axis_length, steer_off]
#kinematic_parameters = np.array([0.1, 0.0106141, 1.4, 0]) #[Ks, Kt, axis_length, steer_off]

# set the laser pose w.r.t robot reference frame
laser_pos_wrt_robot = np.array([1.81022, -0.0228018, 0])
laser_rotation_wrt_robot =  np.array([0, 0, -0.00108296, 0.999999])

# initialize the front wheel configuration
init_front_pose = np.array([1.54757, 0, 0, new_psi_from_abs_enc(encoders_values[0, 0], max_enc_values[0], kinematic_parameters)]) #[x, y, theta, phi]

# rounds_number = 1
# batch_size = 300
# batches_number = math.floor(laser_odometry.shape[0]/ batch_size)
# epsilon = 1e-4
# dx = np.zeros(4)
# print(kinematic_parameters)
# for round_idx in range(rounds_number):
#     for batch_idx in range(batches_number):
#         first_sample_idx = batch_idx*batch_size
#         last_sample_idx = (batch_idx+1)*batch_size
#         laser_odometries_added_dx = []
#         laser_odometries_subtracted_dx = []
#         predicted_front_wheel_odometry = compute_front_wheel_odometry(init_front_pose, kinematic_parameters, encoders_values, max_enc_values)
#         predicted_laser_odometry = compute_laser_odometry(kinematic_parameters, predicted_front_wheel_odometry, laser_pos_wrt_robot, laser_rotation_wrt_robot)
#         curr_init_front_pose = predicted_front_wheel_odometry[first_sample_idx, :]
#         for i in range(dx.shape[0]):
#             dx[i] = epsilon
#             front_wheel_odometry_added_dx = compute_front_wheel_odometry(curr_init_front_pose, + dx + kinematic_parameters, encoders_values[first_sample_idx:last_sample_idx, :], max_enc_values)
#             front_wheel_odometry_subtracted_dx = compute_front_wheel_odometry(curr_init_front_pose, - dx + kinematic_parameters, encoders_values[first_sample_idx:last_sample_idx, :], max_enc_values)
#             laser_odometry_added_dx = compute_laser_odometry(+ dx + kinematic_parameters, front_wheel_odometry_added_dx, laser_pos_wrt_robot, laser_rotation_wrt_robot)
#             laser_odometry_subtracted_dx = compute_laser_odometry(- dx + kinematic_parameters, front_wheel_odometry_subtracted_dx, laser_pos_wrt_robot, laser_rotation_wrt_robot)
#             dx[i] = 0
#             laser_odometries_added_dx.append(laser_odometry_added_dx)
#             laser_odometries_subtracted_dx.append(laser_odometry_subtracted_dx)
#         laser_odometries_added_dx = np.array(laser_odometries_added_dx)
#         laser_odometries_subtracted_dx = np.array(laser_odometries_subtracted_dx)
#         plt.plot(laser_odometries_added_dx[0, :, 0], laser_odometries_added_dx[0, :, 1])
#         plt.plot(laser_odometries_subtracted_dx[0, :, 0], laser_odometries_subtracted_dx[0, :, 1])
#         plt.show()
#         h_x_array, z_array, error_array, kinematic_parameters = ls_calibrate_odometry(kinematic_parameters, laser_odometry[first_sample_idx:last_sample_idx, :], predicted_laser_odometry[first_sample_idx:last_sample_idx, :], laser_odometries_added_dx, laser_odometries_subtracted_dx)

#         print(kinematic_parameters)



rounds_number = 1
batch_size = 300
batches_number = math.floor(laser_odometry.shape[0]/ batch_size)
epsilon = 1e-4
dx = np.zeros(4)
print(kinematic_parameters)
for round_idx in range(rounds_number):
    curr_init_front_pose = init_front_pose
    for batch_idx in range(batches_number):
        first_sample_idx = batch_idx*batch_size - batch_idx
        last_sample_idx = (batch_idx+1)*batch_size - (batch_idx+1)
        laser_odometries_added_dx = []
        laser_odometries_subtracted_dx = []
        predicted_front_wheel_odometry = compute_front_wheel_odometry(init_front_pose, kinematic_parameters, encoders_values, max_enc_values)
        predicted_laser_odometry = compute_laser_odometry(kinematic_parameters, predicted_front_wheel_odometry, laser_pos_wrt_robot, laser_rotation_wrt_robot)
        curr_init_front_pose = predicted_front_wheel_odometry[first_sample_idx, :]
        for i in range(dx.shape[0]):
            dx[i] = epsilon
            front_wheel_odometry_added_dx = compute_front_wheel_odometry(curr_init_front_pose, + dx + kinematic_parameters, encoders_values[first_sample_idx:last_sample_idx+1, :], max_enc_values)
            front_wheel_odometry_subtracted_dx = compute_front_wheel_odometry(curr_init_front_pose, - dx + kinematic_parameters, encoders_values[first_sample_idx:last_sample_idx+1, :], max_enc_values)
            laser_odometry_added_dx = compute_laser_odometry(+ dx + kinematic_parameters, front_wheel_odometry_added_dx, laser_pos_wrt_robot, laser_rotation_wrt_robot)
            laser_odometry_subtracted_dx = compute_laser_odometry(- dx + kinematic_parameters, front_wheel_odometry_subtracted_dx, laser_pos_wrt_robot, laser_rotation_wrt_robot)
            dx[i] = 0
            laser_odometries_added_dx.append(laser_odometry_added_dx)
            laser_odometries_subtracted_dx.append(laser_odometry_subtracted_dx)
        laser_odometries_added_dx = np.array(laser_odometries_added_dx)
        laser_odometries_subtracted_dx = np.array(laser_odometries_subtracted_dx)
        plt.plot(laser_odometries_added_dx[0, :, 0], laser_odometries_added_dx[0, :, 1])
        plt.plot(laser_odometries_subtracted_dx[0,:, 0], laser_odometries_subtracted_dx[0,:, 1])
        plt.show()
        h_x_array, z_array, error_array, kinematic_parameters = ls_calibrate_odometry(kinematic_parameters, laser_odometry[first_sample_idx:last_sample_idx+1, :], predicted_laser_odometry[first_sample_idx:last_sample_idx+1, :], laser_odometries_added_dx, laser_odometries_subtracted_dx, first_sample_idx, last_sample_idx)
        print(kinematic_parameters)

# predicted_front_wheel_odometry = compute_front_wheel_odometry(init_front_pose, kinematic_parameters, encoders_values, max_enc_values)
# predicted_laser_odometry = compute_laser_odometry(kinematic_parameters, predicted_front_wheel_odometry, laser_pos_wrt_robot, laser_rotation_wrt_robot)
# num_points = -1
# plt.plot(laser_odometry[0:num_points, 0], laser_odometry[0:num_points, 1])
# plt.plot(predicted_laser_odometry[0:num_points, 0], predicted_laser_odometry[0:num_points, 1])
# plt.show()
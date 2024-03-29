#--------------------------------------------------------------------------------------------

from dataset_handler import *
from utility import *
from front_tractor_tricycle import *
from ls_odometry_calibration import *
from graphic_representation import *
import numpy as np
import math
import matplotlib.pyplot as plt
from time import sleep

#---------------------------Create Clean and Consistent Dataset------------------------------

# IMPORTANTE NOTE: 
# Uncomment the following part if "consistent_dataset.txt" does not exist in the Datasets folder

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

# Initializes the main system variables
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

# Initialize the kinematic parameters
# IMPORTANT NOTE                 [0]      [1]        [2]          [3]          [4]            [5]              [6]
# kinematic parameters =        [ Ks | Kt       | axis_length| steer_off| robot_x_laser| robot_y_laser| robot_theta_laser]
kinematic_parameters = np.array([ 0.1, 0.0106141, 1.4        , 0        , 1.5          , 0            , 0          ]) 

# Set the laser pose w.r.t robot reference frame
laser_pos_wrt_robot = np.array([1.81022, -0.0228018, 0])
laser_rotation_wrt_robot =  np.array([0, 0, -0.00108296, 0.999999])

#--------------------------------------------------------------------------------------------

# Read from clened and consistent dataset
[num_records, timestamp, encoders_values, robot_odometry_with_initial_guess, laser_odometry] = read_from_consistent_dataset(consistent_dataset_path, info_separator, timestamp_name, encoders_values_name, robot_pose_name, laser_pose_name)

# Check that all dimensions are consistent
dimensions_sanity_checks(num_records, num_encoders, dim_robot_pose_space, dim_laser_pose_space, timestamp, encoders_values, robot_odometry_with_initial_guess, laser_odometry)


# Initialize the front wheel configuration
init_front_pose = np.array([1.54757, 0, 0, new_psi_from_abs_enc(encoders_values[0, 0], max_enc_values[0], kinematic_parameters)]) #[x, y, theta, psi]

# Initialize plot
fig, ax = plt.subplots(nrows=2, ncols=1)
predicted_xy_laser_plot, predicted_theta_laser_plot = initialize_plot(fig, ax, laser_odometry)

#--------------------------------------------------------------------------------------------
# First training in one round, on the whole dataset divided into batches, to approach the minimum

# Initialize hyperparameter of least square method and fit
batches_number = 10
batch_size = math.floor(laser_odometry.shape[0]/batches_number)
epsilon = 1e-3
rounds_number = 1
kinematic_parameters = fit(epsilon, batch_size, batches_number, rounds_number, ax, laser_odometry, kinematic_parameters, init_front_pose, encoders_values, max_enc_values, predicted_xy_laser_plot, predicted_theta_laser_plot)

#--------------------------------------------------------------------------------------------
# Second training in many rounds, on the whole dataset as batch, to achieve the minimum

# Initialize hyperparameter of least square method and fit
epsilon = 1e-4
batch_size = laser_odometry.shape[0] #all dataset in one batch
batches_number = math.floor(laser_odometry.shape[0]/batch_size)
rounds_number = 6
kinematic_parameters = fit(epsilon, batch_size, batches_number, rounds_number, ax, laser_odometry, kinematic_parameters, init_front_pose, encoders_values, max_enc_values, predicted_xy_laser_plot, predicted_theta_laser_plot)
print("end of calibration")
plt.show()
#--------------------------------------------------------------------------------------------
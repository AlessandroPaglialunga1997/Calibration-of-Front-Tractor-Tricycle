#----------------------------------------------------------------------------------------------------

import numpy as np

#--------------------------------------------------------------------------------------------
# Clean the dataset by removing comments and replacing consecutive spaces with a single space

def remove_unnecessary_comments_and_spaces(old_dataset_path, #original dataset
                                           new_dataset_path, #dataset without comments and unnecessary spaces
                                           comment_symbol): #if a line starts with this symbol then it is a comment
    old_dataset_file = open(old_dataset_path) # open the original dataset file
    new_dataset_file = open(new_dataset_path,'w') # open or create a new (clean) dataset file
    all_lines = old_dataset_file.read().splitlines()
    for line in all_lines: #check whether a line is a comment or contains unnecessary spaces
        if line[0] == comment_symbol: #I assume that in the case where a line was a comment then the first character would be the comment symbol
            continue #ingnore comment
        else:
            all_tokens = line.split(" ") 
            new_line = '' #initilize new line
            for token in all_tokens:
                if token == '': #from two consecutive spaces split(" ") returns a empty character
                    continue #ignore empty character
                else:
                    new_line = new_line + token + " " #cat
            new_line = new_line[:-1] #remove last space added
            new_dataset_file.write(new_line +'\n') #append new line in the new (clean) dataset

#----------------------------------------------------------------------------------------------------
# Make the dataset consistent by resetting the timestamp and incremental 
# encoder value from first record (considering variable overflow) 
# Note: [inc_enc = Incremental Encoder]

def make_dataset_consistent(old_dataset_path, #clean dataset
                            new_dataset_path, #consistent dataset path
                            info_separator, #each vector (info) is preceded by a name and a separator (e.g. time: ':' is the separator)
                            timestamp_name,
                            encoders_info_name,
                            first_timestamp,
                            first_inc_enc, # first incremental encoder value
                            overflow_value): # maximum absolute variable value (e.g. unit32 max value = 4294967295)
    old_dataset_file = open(old_dataset_path) # open the original dataset file
    new_dataset_file = open(new_dataset_path,'w') # open or create a new (consistent) dataset file
    all_lines = old_dataset_file.read().splitlines()
    previous_inc_enc = first_inc_enc
    new_inc_enc = 0 #the incremental encoder value can start from 0 because during odometry only the difference between two consecutive incremental encoder value is used
    previous_time = first_timestamp
    new_time = 0 #reset the timestamp for the same reason of incremental encoder value
    for line in all_lines:
        all_tokens = line.split(" ")
        index_token = 0
        new_line = '' #initilize new line
        for token in all_tokens:
            new_line = new_line + token + " "
            if token.lower() == timestamp_name + info_separator: #if the current token is "time:" then the second one is its value
                current_time = float(all_tokens[index_token + 1]) 
                delta_time = current_time - previous_time #time difference between two consecutive records (lines)
                new_time += delta_time #update the new timestamp
                all_tokens[index_token + 1] = str('%.6f'%(new_time)) #update the token related to the timestamp value
                previous_time = current_time #update previous_time for next line
            elif token.lower() == encoders_info_name + info_separator: #if the current token is "ticks:" then the third one is incremental encoder value
                current_inc_enc = int(all_tokens[index_token + 2]) 
                ticks_without_overflow = current_inc_enc - previous_inc_enc #in the absence of overflow the ticks made between one timestamp and the next is given by the current value minus the previous value of the incremental encoder. This difference can also be negative (the robot is going backwards).
                ticks_with_overflow = overflow_value - previous_inc_enc + current_inc_enc #in the presence of overflow the ticks made between one timestamp and the next is given by the maximum value of the variable used (e.g. uint32) minus the previous value plus the current value of the incremental encoder.
                if current_inc_enc < previous_inc_enc and ticks_with_overflow < previous_inc_enc - current_inc_enc: #if the current value is less than the previous value of the incremental encoder, then considering the maximum value of the variable used (e.g. uint32), it is absurd that the ticks made in the presence of overflow are greater than the difference between the previous value and the current value of the incremental encoder, so we are in the presence of an overflow
                    new_inc_enc = int(new_inc_enc + ticks_with_overflow)
                else: 
                    new_inc_enc = int(new_inc_enc + ticks_without_overflow)
                previous_inc_enc = current_inc_enc #update previous_inc_enc for next line
                all_tokens[index_token + 2] = str(new_inc_enc) #update the token related to the incremental encoder value value
            index_token += 1
        new_line = new_line[:-1] #cancel last unnecessary space 
        new_dataset_file.write(new_line +'\n') #append new line in the new (consistent) dataset

#--------------------------------------------------------------------------------------------
# Given a clean and consistent dataset file it returns a numpy array for each column
# In this project the columns are the following one:
# timestamp
# encoders values: absolute and incremental encoders values
# robot odometry: robot odometry trajectory with initial guess
# laser odometry: laser odometry trajectory with calibrated parameters

def read_from_consistent_dataset(consistent_dataset_path, 
                                 info_separator, 
                                 timestamp_name, 
                                 encoders_values_name, 
                                 robot_odom_name, 
                                 laser_odom_name):
    timestamp = []
    encoders_values = []
    robot_odom = []
    laser_odom = []
    consistent_dataset_file = open(consistent_dataset_path)
    all_lines = consistent_dataset_file.read().splitlines()
    num_records = len(all_lines)
    for line in all_lines:
        all_tokens = line.split(" ")
        index_token = 0
        for token in all_tokens:
            if token.lower() == timestamp_name + info_separator: #if the current token is "time:" then the second one is its value
                timestamp.append(float(all_tokens[index_token + 1]))
            elif token.lower() == encoders_values_name + info_separator: #if the current token is "ticks:" then second one is absolute encoder value while third one is incremental encoder value
                abs_enc_value = int(all_tokens[index_token + 1])
                inc_enc_value = int(all_tokens[index_token + 2])
                curr_encoders_value_np = np.array([abs_enc_value, inc_enc_value])
                encoders_values.append(curr_encoders_value_np)
            elif token.lower() == robot_odom_name + info_separator: #if the current token is "model_pose:" then second, third and fourth one is robot pose (x, y, theta)
                x = float(all_tokens[index_token + 1])
                y = float(all_tokens[index_token + 2])
                theta = float(all_tokens[index_token + 3])
                curr_robot_odom_np = np.array([x, y, theta])
                robot_odom.append(curr_robot_odom_np)
            elif token.lower() == laser_odom_name + info_separator: #if the current token is "laser_pose:" then second, third and fourth one is laser pose (x, y, theta)
                x = float(all_tokens[index_token + 1])
                y = float(all_tokens[index_token + 2])
                theta = float(all_tokens[index_token + 3])
                curr_laser_odom_np = np.array([x, y, theta])
                laser_odom.append(curr_laser_odom_np)
            index_token += 1
    return [num_records, np.array(timestamp), np.array(encoders_values), np.array(robot_odom), np.array(laser_odom)]

#--------------------------------------------------------------------------------------------

def dimensions_sanity_checks(num_records, num_encoders, dim_robot_odom_space, dim_laser_odom_space, timestamp, encoders_values, robot_odom, laser_odom):
    assert timestamp.shape[0] == num_records
    assert encoders_values.shape[0] == num_records
    assert encoders_values.shape[1] == num_encoders
    assert robot_odom.shape[0] == num_records
    assert robot_odom.shape[1] == dim_robot_odom_space
    assert laser_odom.shape[0] == num_records
    assert laser_odom.shape[1] == dim_laser_odom_space

#--------------------------------------------------------------------------------------------

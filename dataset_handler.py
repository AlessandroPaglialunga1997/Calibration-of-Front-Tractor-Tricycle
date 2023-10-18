#----------------------------------------------------------------------------------------------------
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
# encoder value from first record (considering variable overflow) [inc_enc = Incremental Encoder]

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
        new_line = ''
        for token in all_tokens:
            new_line = new_line + token + " "
            if token.lower() == timestamp_name + info_separator:
                current_time = float(all_tokens[index_token + 1]) #if the current token is e.g. "time:" then the second one is its value
                delta_time = current_time - previous_time #time difference between two consecutive records (lines)
                new_time += delta_time #update the new timestamp
                all_tokens[index_token + 1] = str('%.6f'%(new_time)) #update the token related to the timestamp value
                previous_time = current_time #update previous_time for next line
            elif token.lower() == encoders_info_name + info_separator:
                current_inc_enc = int(all_tokens[index_token + 2]) #if the current token is e.g. "ticks:" then the third one is incremental encoder value
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
        
# Clean the dataset by removing comments and replacing consecutive spaces with a single space

def remove_unnecessary_comments_and_spaces(old_dataset_path, #original dataset
                                           new_dataset_path, #dataset without comments and unnecessary spaces
                                           comment_symbol): #if a line starts with this symbol then it is a comment
    old_dataset_file = open(old_dataset_path) # open the original dataset file
    new_dataset_file = open(new_dataset_path,'w') # open or create a new (cleaned) dataset file
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
            new_dataset_file.write(new_line +'\n') #append new line in the new (cleaned) dataset

# Make the dataset consistent by resetting the timestamp and incremental 
# encoder value from first record (considering variable overflow)


def make_dataset_consistent(old_dataset_path, #cleaned dataset
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
    previous_time = first_timestamp
    previous_inc_enc = first_inc_enc
    new_inc_enc = 0
    for line in all_lines:
        all_tokens = line.split(" ")
        index_token = 0
        new_line = ''
        for token in all_tokens:
            new_line = new_line + token + " "
            if token.lower() == timestamp_name + info_separator:
                current_time = float(all_tokens[index_token + 1])
                new_time = '%.6f'%(current_time - previous_time)
                previous_time = current_time
                all_tokens[index_token + 1] = str(new_time)
            elif token.lower() == encoders_info_name + info_separator:
                current_inc_enc = int(all_tokens[index_token+2])
                ticks_without_overflow = current_inc_enc - previous_inc_enc
                ticks_with_overflow = overflow_value - previous_inc_enc + current_inc_enc
                if current_inc_enc < previous_inc_enc and ticks_with_overflow < previous_inc_enc - current_inc_enc:
                    new_inc_enc = int(new_inc_enc + ticks_with_overflow)
                else: 
                    new_inc_enc = int(new_inc_enc + ticks_without_overflow)
                previous_inc_enc = current_inc_enc
                all_tokens[index_token + 2] = str(new_inc_enc)
            index_token += 1
        new_line = new_line[:-1]
        new_dataset_file.write(new_line +'\n')
        
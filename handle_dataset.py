# DS stands for DataSet
# infos stands for informations

from utilities import *
import numpy as np

def clean_dataset_from_unuseful_spaces(DS_path, num_comments, new_DS_path):
    DS_file = open(DS_path)
    new_DS_file = open(new_DS_path,'w')
    lines = DS_file.read().splitlines()
    for line_idx in range(num_comments, len(lines)):
        there_was_a_space = False
        char_idx = 0
        while char_idx < len(lines[line_idx]):
            if(lines[line_idx][char_idx] == " "):
                if(there_was_a_space):
                    lines[line_idx] = lines[line_idx][:char_idx] + lines[line_idx][char_idx+1:]
                    char_idx -= 1
                else:
                    there_was_a_space = True
            else:
                there_was_a_space = False
            char_idx += 1
        new_DS_file.write(lines[line_idx]+'\n')

def read_DS(DS_path, num_comments, separator):
    num_records = num_of_lines_in_file(DS_path) - num_comments
    num_tokens = num_of_tokens_in_line(DS_path, num_comments, separator) - 1 #last one contains only numbers
    DS_file = open(DS_path)
    [num_infos_per_token_array, total_num_of_infos] = tokens_dimensions(DS_path, separator, num_comments, num_tokens)

    data = take_data(DS_path, separator, num_records, num_comments, num_infos_per_token_array, total_num_of_infos)
    return data

def tokens_dimensions(DS_path, separator, num_comments, num_tokens):
    token_name_array = []
    num_infos_per_token_array = {}
    total_num_of_infos = 0
    DS_file = open(DS_path)
    lines = DS_file.read().splitlines()
    for token_idx in range(0, num_tokens):
        token_name = take_one_word_from_line(lines[num_comments], separator, token_idx, -1)
        token_name_array.append(token_name)
        num_infos = num_of_words_in_token(DS_path, num_comments, separator, token_idx+1)
        if (token_idx < num_tokens - 1): # except for last token 
            num_infos -= 1 # last one is next token name
        num_infos_per_token_array[token_name] = [num_infos, total_num_of_infos]
        total_num_of_infos += num_infos
    
    return [num_infos_per_token_array, total_num_of_infos]

def take_data(DS_path, separator, num_records, num_comments, num_infos_per_token_array, total_num_of_infos):
    DS_file = open(DS_path)
    lines = DS_file.read().splitlines()
    data = np.empty((num_records, total_num_of_infos), dtype='float64')
    for line_idx in range(num_comments, len(lines)):
        tokens = tokens_in_line(DS_path, line_idx, separator)
        record_idx = line_idx - num_comments
        for token_idx in range(1, len(tokens)): #first one is a token name for sure ;)
            token_name = take_one_word_from_token(tokens[token_idx-1], -1)
            for info_idx in range(0, num_infos_per_token_array[token_name][0]):
                column_idx = num_infos_per_token_array[token_name][1] + info_idx
                data[record_idx][column_idx] = float(take_one_word_from_token(tokens[token_idx], info_idx))
    return data
    
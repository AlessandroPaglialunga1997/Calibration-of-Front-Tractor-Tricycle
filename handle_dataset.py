# DS stands for DataSet
# infos stands for informations

from utilities import *
import numpy as np

def read_DS(DS_path, num_comments, separator):
    num_records = num_of_lines_in_file(DS_path) - num_comments
    num_tokens = num_of_tokens_in_line(DS_path, num_comments, separator) - 1 #last one contains only numbers
    token_name_array = []
    num_infos_per_token_array = {}
    total_num_of_infos = 0
    f = open(DS_path)
    lines = f.read().splitlines()
    for token_idx in range(0, num_tokens):
        token_name = take_one_word_from_line(lines[num_comments], separator, token_idx, -1)
        token_name_array.append(token_name)
        num_infos = num_of_words_in_token(DS_path, num_comments, separator, token_idx+1)
        if (token_idx < num_tokens - 1): # except for last token 
            num_infos -= 1 # last one is next token name
        num_infos_per_token_array[token_name] = num_infos
        total_num_of_infos += num_infos
    
    data = np.array((num_records, total_num_of_infos))
    for line_idx in range(num_comments, len(lines)):
        
    
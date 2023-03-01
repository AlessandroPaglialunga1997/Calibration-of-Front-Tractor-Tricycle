import sys
sys.path.insert(1, '../utility/')
from utilities import *
from file_handler import *
from dataset_handler import *
from decimal import Decimal

# for this application used data are stored in 'new_DS_path' but they must be consistet
# with the given dataset 'DS_path' otherwise data in 'new_DS_path' will be recomputed

def dataset_validation(new_DS_path, DS_path, num_comments, separator, unchanged_column_idx, string_new_data_value):
    [num_infos_per_token_array, string_data_value] = read_DS(DS_path, num_comments, separator)
    compare_subset_outcome = compare_matrices(string_data_value[:, unchanged_column_idx], string_new_data_value[:, unchanged_column_idx])
    
    FIRST_TS = Decimal(take_one_word_from_file(DS_path, num_comments, separator, 1, 0))
    translated_TS_array = take_column_from_file(new_DS_path, 0)
    TS_array = take_column_from_file(DS_path, 1)
    TS_validation_outcome = TS_validation(TS_array, translated_TS_array, FIRST_TS)
    
    FIRST_IncEncInfo = int(take_one_word_from_file(DS_path, num_comments, separator, 2, 1))
    translated_IncEncInfo_array = take_column_from_file(new_DS_path, 2)
    IncEncInfo_array = take_column_from_file(DS_path, 4)
    IncEncInfo_validation_outcome = IncEncInfo_validation(IncEncInfo_array, translated_IncEncInfo_array, FIRST_IncEncInfo)
    
    if (compare_subset_outcome and TS_validation_outcome and IncEncInfo_validation_outcome):
        return [True, string_data_value]
    else:
        return [False, string_data_value]

def TS_validation(TS_array, translated_TS_array, FIRST_TS):
    for i in range(0, len(TS_array)):
        real_TS = round(Decimal(TS_array[i]), 5)
        translated_TS = round(Decimal(translated_TS_array[i]) + FIRST_TS, 5)
        if(real_TS != translated_TS):
            return False
    return True

def IncEncInfo_validation(IncEncInfo_array, translated_IncEncInfo_array, FIRST_IncEncInfo):
    max_value_of_uint32 = 4294967295
    pre_IncEncInfo = FIRST_IncEncInfo
    pre_translated_IncEncInfo = int(translated_IncEncInfo_array[0])
    for i in range(0, len(IncEncInfo_array)):
        real_IncEncInfo = int(IncEncInfo_array[i])
        cur_translated_IncEncInfo = int(translated_IncEncInfo_array[i])
        tick_steps = cur_translated_IncEncInfo - pre_translated_IncEncInfo
        estimated_cur_IncEncInfo = pre_IncEncInfo + tick_steps
        if(estimated_cur_IncEncInfo > max_value_of_uint32):
            estimated_cur_IncEncInfo -= max_value_of_uint32 
        if(estimated_cur_IncEncInfo != real_IncEncInfo):
            return False
        pre_translated_IncEncInfo = cur_translated_IncEncInfo
        pre_IncEncInfo = real_IncEncInfo
    return True
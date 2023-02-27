from utilities import *
from file_handler import *
from dataset_handler import *
from decimal import Decimal

# for this application used data are stored in 'new_DS_path' but they must be consistet
# with the given dataset 'DS_path' otherwise data in 'new_DS_path' will be recomputed

def dataset_validation(new_DS_path, DS_path, num_comments, separator, unchanged_column_idx):
    # read data from new_DS_path
    [num_infos_per_token_array, string_data_value] = read_DS(DS_path, num_comments, separator)
    [read_new_DS_outcome, string_new_data_value] = read_new_DS(new_DS_path)
    if (read_new_DS_outcome == False):
        return [False, string_data_value, None]
    else:
        compare_subset_outcome = compare_matrices(string_data_value[:, unchanged_column_idx], string_new_data_value[:, unchanged_column_idx])
        string_FIRST_TS = take_one_word_from_file(DS_path, num_comments, separator, 1, 0) 
        FIRST_TS = Decimal(string_FIRST_TS)
        translated_TS_array = take_column_from_file(new_DS_path, 0)
        TS_validation_outcome = TS_validation(DS_path, num_comments, separator, translated_TS_array, FIRST_TS)
        IncEncInfo_validation_outcome = True
        if (compare_subset_outcome and TS_validation_outcome and IncEncInfo_validation_outcome):
            return [True, string_data_value, string_new_data_value]
        else:
            return [False, string_data_value, None]

def TS_validation(DS_path, num_comments, separator, translated_TS_array, FIRST_TS):
	f = open(DS_path)
	lines = f.read().splitlines()
	for line_idx in range(num_comments, len(lines)):
		TS_idx = line_idx - num_comments
		cur_TS = float(take_one_word_from_line(lines[line_idx], separator, 1, 0))
		if (translated_TS_array[TS_idx] + FIRST_TS != cur_TS):
			return False
	return True

def IncEncInfo_validation(DS_path, num_comments, separator, translated_IncEncInfo_array, FIRST_IncEncInfo):
    max_value_of_uint32 = 4294967295
    f = open(DS_path)
    lines = f.read().splitlines()
    pre_IncEncInfo = FIRST_IncEncInfo
    pre_translated_IncEncInfo = translated_IncEncInfo_array[0]
    for line_idx in range(num_comments, len(lines)):
        TS_idx = line_idx - num_comments
        cur_IncEncInfo = int(take_one_word_from_line(lines[line_idx], separator, 2, 1))
        cur_translated_IncEncInfo = translated_IncEncInfo_array[TS_idx]
        tick_steps = cur_translated_IncEncInfo - pre_translated_IncEncInfo
        estimated_cur_IncEncInfo = pre_IncEncInfo + tick_steps
        if(estimated_cur_IncEncInfo > max_value_of_uint32):
            estimated_cur_IncEncInfo -= max_value_of_uint32 
        if(estimated_cur_IncEncInfo != cur_IncEncInfo):
            return False
        pre_translated_IncEncInfo = cur_translated_IncEncInfo
        pre_IncEncInfo = cur_IncEncInfo
    return True
from utilities import *
from handle_dataset import *

# for this application used data are stored in 'new_DS_path' but they must be consistet
# with the given dataset 'DS_path' otherwise data in 'new_DS_path' will be recomputed

def dataset_validation(new_DS_path, DS_path, num_comments, separator):
    # read data from new_DS_path
    [num_infos_per_token_array, string_data_value] = read_DS(DS_path, num_comments, separator)
    [outcome, new_data] = read_new_DS(new_DS_path)
    if (outcome == False):
        print("I didn't read correctly new dataset")
        return [False, string_data_value]
    else:
        print("I read correctly new dataset")
        # compare dataset
        
        return [True, string_data_value]

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
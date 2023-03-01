# TS stands for TimeStamp
# DS stands for DataSet
# num stands for number
# pre stands for previous
# cur stands fro current
 
from file_handler import *
import numpy as np

def reset_TS(DS_path, num_comments, separator):
	string_FIRST_TS = take_one_word_from_file(DS_path, num_comments, separator, 1, 0) 
	FIRST_TS = float(string_FIRST_TS)
	num_decimals = len(string_FIRST_TS.split(".")[1])
	pre_TS = FIRST_TS
	
	f = open(DS_path)
	lines = f.read().splitlines()
	translated_TS_array = []
	time_sampling_array = []
	for line_idx in range(num_comments, len(lines)):
		cur_TS = float(take_one_word_from_line(lines[line_idx], separator, 1, 0))
		[translated_cur_TS, time_sampling] = translate_TS_and_compute_time_sampling(cur_TS, FIRST_TS, pre_TS)
		translated_TS_array.append(round(translated_cur_TS, num_decimals))
		time_sampling_array.append(round(time_sampling, num_decimals))
		pre_TS = cur_TS
	return np.asarray(translated_TS_array)

def translate_TS_and_compute_time_sampling(cur_TS, FIRST_TS, pre_TS):
	translated_cur_TS = cur_TS - FIRST_TS
	time_sampling = cur_TS - pre_TS
	return [translated_cur_TS, time_sampling]
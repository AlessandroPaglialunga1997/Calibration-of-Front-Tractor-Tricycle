import numpy as np
from utilities import *

def translate_timestamp(dataset_path, number_of_comments, separator):
	FIRST_timestamp = float(take_one_word_from_file(dataset_path, number_of_comments, separator, 1, 0))
	previous_timestamp = FIRST_timestamp
	
	f = open(dataset_path)
	lines = f.read().splitlines()
	translated_timestamp_array = []
	time_sampling_array = []
	for i in range(number_of_comments, len(lines)):
		current_timestamp = float(take_one_word_from_line(lines[i], separator, 1, 0))
		[translated_current_timestamp, time_sampling] = translated_timestamp_and_time_sampling(current_timestamp, FIRST_timestamp, previous_timestamp)
		translated_timestamp_array.append(translated_current_timestamp)
		time_sampling_array.append(time_sampling)
		previous_timestamp = current_timestamp

	avg_time_sampling = np.mean(np.asarray(time_sampling_array))
	number_of_records = len(translated_timestamp_array)
	print("AVG Time Sampling: %.9f" %(avg_time_sampling))
	print("Num of Records: %d" %(number_of_records))

def translated_timestamp_and_time_sampling(current_timestamp, FIRST_timestamp, previous_timestamp):
	translated_current_timestamp = current_timestamp - FIRST_timestamp
	time_sampling = current_timestamp - previous_timestamp
	print("TS: %.9f; Translated TS: %.9f; Time Sampling: %.9f" % (current_timestamp, translated_current_timestamp, time_sampling))
	return [translated_current_timestamp, time_sampling]
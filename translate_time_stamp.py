import numpy as np
def translate_time_stamp():
	f = open("dataset.txt")
	lines = f.read().splitlines()
	number_of_comments = 8 # there are several lines of comments in this file

	translated_time_stamp_array = []
	time_sampling_array = []

	FIRST_token = lines[number_of_comments].split(":") # first record is after comments
	FIRST_time_stamp_and_ticks_string = FIRST_token[1].strip() # time stamp info is the second element
	FIRST_time_stamp_and_ticks_string_array = FIRST_time_stamp_and_ticks_string.split(" ")
	FIRST_time_stamp = float(FIRST_time_stamp_and_ticks_string_array[0]) #[time stamp; "ticks"]

	previous_time_stamp = FIRST_time_stamp

	c = 0
	for l in lines:
		if(c < number_of_comments): #Jump comments
			c += 1
			continue
		tokens = l.split(":")
		time_stamp_and_ticks_string = tokens[1].strip() # time stamp info is the second element
		time_stamp_and_ticks_string_array = time_stamp_and_ticks_string.split(" ")
		current_time_stamp = float(time_stamp_and_ticks_string_array[0]) #[time stamp; "ticks"]
		translated_current_time_stamp = current_time_stamp - FIRST_time_stamp
		translated_time_stamp_array.append(translated_current_time_stamp)
		time_sampling = current_time_stamp - previous_time_stamp
		time_sampling_array.append(time_sampling)
		previous_time_stamp = current_time_stamp
		print("TS: %.9f; Translated TS: %.9f; Time Sampling: %.9f" % (current_time_stamp, translated_current_time_stamp, time_sampling))

	avg_time_sampling = np.mean(np.asarray(time_sampling_array))
	number_of_records = len(translated_time_stamp_array)
	print("AVG Time Sampling: %.9f" %(avg_time_sampling))
	print("Num of Records: %d" %(number_of_records))


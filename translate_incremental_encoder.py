# AbsEncInfo stands for Absolut Encoder Information
# IncEncInfo stands for Incremental Encoder Information

import numpy as np

def translate_incremental_encoder():
    f = open("dataset.txt")
    lines = f.read().splitlines()
    number_of_comments = 8 # there are several lines of comments in this file

    translated_IncEncInfo_array = []
    ticks_step_btw_two_time_stamp_array = []

    FIRST_token = lines[number_of_comments].split(":") # first record is after comments
    FIRST_AbsEncInfo_and_IncEncInfo = FIRST_token[2].strip() # IncEncInfo is the fourth element
    FIRST_AbsEncInfo_and_IncEncInfo_array = FIRST_AbsEncInfo_and_IncEncInfo.split(" ")
    FIRST_IncEncInfo = int(FIRST_AbsEncInfo_and_IncEncInfo_array[1]) #[AbsEncInfo, IncEncInfo]

    previous_IncEncInfo = FIRST_IncEncInfo
    translated_IncEncInfo = 0
    max_value_of_uint32 = 4294967295

    c = 0
    for l in lines:
        if(c < number_of_comments): #Jump comments
            c += 1
            continue
        tokens = l.split(":")
        AbsEncInfo_and_IncEncInfo_string = tokens[2].strip() # IncEncInfo is the third element
        AbsEncInfo_and_IncEncInfo_string_array = AbsEncInfo_and_IncEncInfo_string.split(" ")
        current_IncEncInfo = int(AbsEncInfo_and_IncEncInfo_string_array[1]) #[AbsEncInfo, IncEncInfo]
        
        ticks_step_btw_two_time_stamp_WITHOUT_OVERFLOW = current_IncEncInfo - previous_IncEncInfo
        ticks_step_btw_two_time_stamp_WITH_OVERFLOW = (max_value_of_uint32 - previous_IncEncInfo) + current_IncEncInfo
        if(current_IncEncInfo < previous_IncEncInfo and ticks_step_btw_two_time_stamp_WITH_OVERFLOW < previous_IncEncInfo - current_IncEncInfo): # variable overflow
            ticks_step = ticks_step_btw_two_time_stamp_WITH_OVERFLOW
            translated_IncEncInfo += ticks_step
            print("Variable Overflow")
        else:
            ticks_step = ticks_step_btw_two_time_stamp_WITHOUT_OVERFLOW
            translated_IncEncInfo += ticks_step
        ticks_step_btw_two_time_stamp_array.append(ticks_step)
        translated_IncEncInfo_array.append(translated_IncEncInfo)
        previous_IncEncInfo = current_IncEncInfo
        print("IncEncInfo: %d; Translated IncEncInfo: %d; Ticks Step: %d" % (current_IncEncInfo, translated_IncEncInfo, ticks_step))

    avg_ticks_step = np.mean(np.asarray(ticks_step_btw_two_time_stamp_array))
    number_of_records = len(translated_IncEncInfo_array)
    print("AVG Ticks Step: %.9f" %(avg_ticks_step))
    print("Num of Records: %d" %(number_of_records))
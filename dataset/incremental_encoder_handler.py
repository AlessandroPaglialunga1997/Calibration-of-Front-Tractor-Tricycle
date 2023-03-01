# AbsEncInfo stands for Absolut Encoder Information
# IncEncInfo stands for Incremental Encoder Information
# TS stands for TimeStamp
# DS stands for DataSet
# num stands for number
# infos stands for informations
# pre stands for previous
# cur stands fro current

from file_handler import *
import numpy as np
import math

def translate_IncEncInfo(DS_path, num_comments, separator, IncEnd_MaxValue):
    FIRST_IncEncInfo = int(take_one_word_from_file(DS_path, num_comments, separator, 2, 1))
    f = open(DS_path)
    lines = f.read().splitlines()
    translated_IncEncInfo_array = avoid_uint32_overflow(lines, num_comments, separator, FIRST_IncEncInfo)
    #translated_IncEncInfo_array = avoid_IncEnd_MaxValue_overflow(translated_IncEncInfo_array, IncEnd_MaxValue)
    return np.asarray(translated_IncEncInfo_array)

def avoid_uint32_overflow(lines, num_comments, separator, pre_IncEncInfo):
    translated_IncEncInfo_array = []
    IncEncInfo = 0

    for line_idx in range(num_comments, len(lines)):
        cur_IncEncInfo = int(take_one_word_from_line(lines[line_idx], separator, 2, 1))
        tick_steps = absolute_tick_steps(cur_IncEncInfo, pre_IncEncInfo)
        IncEncInfo += tick_steps 
        if (tick_steps <= -5000 or tick_steps >= 5000):
            print("%d Cazzo" %(line_idx))
        translated_IncEncInfo_array.append(IncEncInfo)
        pre_IncEncInfo = cur_IncEncInfo
    return translated_IncEncInfo_array

def absolute_tick_steps(cur_IncEncInfo, pre_IncEncInfo):
    max_value_of_uint32 = 4294967295
    tick_steps_WITHOUT_OVERFLOW = cur_IncEncInfo - pre_IncEncInfo
    tick_steps_WITH_OVERFLOW = (max_value_of_uint32 - pre_IncEncInfo) + cur_IncEncInfo
    if(cur_IncEncInfo < pre_IncEncInfo and tick_steps_WITH_OVERFLOW < pre_IncEncInfo - cur_IncEncInfo): # variable overflow
        tick_steps = tick_steps_WITH_OVERFLOW
    else:
        tick_steps = tick_steps_WITHOUT_OVERFLOW
    return tick_steps

# def avoid_IncEnd_MaxValue_overflow(translated_IncEncInfo_array, IncEnd_MaxValue):
#     for i in range(0, len(translated_IncEncInfo_array)):
#         cur_IncEncInfo = translated_IncEncInfo_array[i]
#         if (cur_IncEncInfo >= 0 and cur_IncEncInfo <= IncEnd_MaxValue):
#             continue
#         correct_IncEncInfo = cur_IncEncInfo - IncEnd_MaxValue*math.floor(translated_IncEncInfo_array[i]/IncEnd_MaxValue)
#         translated_IncEncInfo_array[i] = correct_IncEncInfo
    
#     return translated_IncEncInfo_array  
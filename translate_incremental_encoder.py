# AbsEncInfo stands for Absolut Encoder Information
# IncEncInfo stands for Incremental Encoder Information
import numpy as np
from utilities import *

def translate_incremental_encoder(dataset_path, number_of_comments, separator):
    FIRST_IncEncInfo = int(take_one_word_from_file(dataset_path, number_of_comments, separator, 2, 1))
    previous_IncEncInfo = FIRST_IncEncInfo

    f = open("dataset.txt")
    lines = f.read().splitlines()
    translated_IncEncInfo_array = []
    tick_steps_array = []
    IncEncInfo = 0

    for i in range(number_of_comments, len(lines)):
        current_IncEncInfo = int(take_one_word_from_line(lines[i], separator, 2, 1))
        tick_steps = absolute_tick_steps(current_IncEncInfo, previous_IncEncInfo)
        IncEncInfo += tick_steps 
        tick_steps_array.append(tick_steps)
        translated_IncEncInfo_array.append(IncEncInfo)
        previous_IncEncInfo = current_IncEncInfo
        print("IncEncInfo: %d; Translated IncEncInfo: %d; Ticks Step: %d" % (current_IncEncInfo, IncEncInfo, tick_steps))

    avg_tick_steps = np.mean(np.asarray(tick_steps_array))
    number_of_records = len(translated_IncEncInfo_array)
    print("AVG Ticks Step: %.9f" %(avg_tick_steps))
    print("Num of Records: %d" %(number_of_records))

def absolute_tick_steps(current_IncEncInfo, previous_IncEncInfo):
    max_value_of_uint32 = 4294967295
    tick_steps_WITHOUT_OVERFLOW = current_IncEncInfo - previous_IncEncInfo
    tick_steps_WITH_OVERFLOW = (max_value_of_uint32 - previous_IncEncInfo) + current_IncEncInfo
    if(current_IncEncInfo < previous_IncEncInfo and tick_steps_WITH_OVERFLOW < previous_IncEncInfo - current_IncEncInfo): # variable overflow
        tick_steps = tick_steps_WITH_OVERFLOW
    else:
        tick_steps = tick_steps_WITHOUT_OVERFLOW
    return tick_steps
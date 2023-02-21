# AbsEncInfo stands for Absolut Encoder Information
# IncEncInfo stands for Incremental Encoder Information
import numpy as np
from utilities import *

def translate_incremental_encoder_informations(dataset_path, number_of_comments, separator):
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
    return [translated_IncEncInfo_array, FIRST_IncEncInfo]

def absolute_tick_steps(current_IncEncInfo, previous_IncEncInfo):
    max_value_of_uint32 = 4294967295
    tick_steps_WITHOUT_OVERFLOW = current_IncEncInfo - previous_IncEncInfo
    tick_steps_WITH_OVERFLOW = (max_value_of_uint32 - previous_IncEncInfo) + current_IncEncInfo
    if(current_IncEncInfo < previous_IncEncInfo and tick_steps_WITH_OVERFLOW < previous_IncEncInfo - current_IncEncInfo): # variable overflow
        tick_steps = tick_steps_WITH_OVERFLOW
    else:
        tick_steps = tick_steps_WITHOUT_OVERFLOW
    return tick_steps

def validate_incremental_encoder_information_translation(dataset_path, number_of_comments, separator, translated_IncEncInfo_array, FIRST_IncEncInfo):
    max_value_of_uint32 = 4294967295
    f = open(dataset_path)
    lines = f.read().splitlines()
    previous_IncEncInfo = FIRST_IncEncInfo
    previous_translated_IncEncInfo = translated_IncEncInfo_array[0]
    for i in range(number_of_comments, len(lines)):
        timestamp_idx = i - number_of_comments
        current_IncEncInfo = int(take_one_word_from_line(lines[i], separator, 2, 1))
        current_translated_IncEncInfo = translated_IncEncInfo_array[timestamp_idx]
        tick_steps = current_translated_IncEncInfo - previous_translated_IncEncInfo
        estimated_current_IncEncInfo = previous_IncEncInfo + tick_steps
        if(estimated_current_IncEncInfo > max_value_of_uint32):
            estimated_current_IncEncInfo -= max_value_of_uint32 
        if(estimated_current_IncEncInfo != current_IncEncInfo):
            return False
        previous_translated_IncEncInfo = current_translated_IncEncInfo
        previous_IncEncInfo = current_IncEncInfo
    return True
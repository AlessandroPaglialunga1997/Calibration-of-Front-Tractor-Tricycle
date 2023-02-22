# AbsEncInfo stands for Absolut Encoder Information
# IncEncInfo stands for Incremental Encoder Information
# TS stands for TimeStamp
# DS stands for DataSet
# num stands for number
# infos stands for informations
# pre stands for previous
# cur stands fro current

from utilities import *
import numpy as np

def translate_IncEncInfo(DS_path, num_comments, separator):
    FIRST_IncEncInfo = int(take_one_word_from_file(DS_path, num_comments, separator, 2, 1))
    pre_IncEncInfo = FIRST_IncEncInfo

    f = open(DS_path)
    lines = f.read().splitlines()
    translated_IncEncInfo_array = []
    tick_steps_array = []
    IncEncInfo = 0

    for line_idx in range(num_comments, len(lines)):
        cur_IncEncInfo = int(take_one_word_from_line(lines[line_idx], separator, 2, 1))
        tick_steps = absolute_tick_steps(cur_IncEncInfo, pre_IncEncInfo)
        IncEncInfo += tick_steps 
        tick_steps_array.append(tick_steps)
        translated_IncEncInfo_array.append(IncEncInfo)
        pre_IncEncInfo = cur_IncEncInfo
    return [np.asarray(translated_IncEncInfo_array), FIRST_IncEncInfo]

def absolute_tick_steps(cur_IncEncInfo, pre_IncEncInfo):
    max_value_of_uint32 = 4294967295
    tick_steps_WITHOUT_OVERFLOW = cur_IncEncInfo - pre_IncEncInfo
    tick_steps_WITH_OVERFLOW = (max_value_of_uint32 - pre_IncEncInfo) + cur_IncEncInfo
    if(cur_IncEncInfo < pre_IncEncInfo and tick_steps_WITH_OVERFLOW < pre_IncEncInfo - cur_IncEncInfo): # variable overflow
        tick_steps = tick_steps_WITH_OVERFLOW
    else:
        tick_steps = tick_steps_WITHOUT_OVERFLOW
    return tick_steps

def val_IncEncInfo_translation(DS_path, num_comments, separator, translated_IncEncInfo_array, FIRST_IncEncInfo):
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
# TS stands for TimeStamp
# DS stands for DataSet
# IncEncInfo stands for Incremental Encoder Information
# num stands for number
# tokens stands for informations

from handle_timestamp import *
from handle_incremental_encoder import *
from handle_dataset import *
import numpy as np

DS_path = "dataset.txt"
new_DS_path = "new_dataset.txt"
num_comments = 8 # in the given dataset
tokens_separator = ":"

#[translated_TS_array, FIRST_TS] = translate_TS(DS_path, num_comments, tokens_separator)
#[translated_IncEncInfo_array, FIRST_IncEncInfo] = translate_IncEncInfo(DS_path, num_comments, tokens_separator)

#TS_val = val_TS_translation(DS_path, num_comments, tokens_separator, translated_TS_array, FIRST_TS)
#IncEncInfo_val = val_IncEncInfo_translation(DS_path, num_comments, tokens_separator, translated_IncEncInfo_array, FIRST_IncEncInfo)

read_DS(DS_path, num_comments, tokens_separator, new_DS_path)
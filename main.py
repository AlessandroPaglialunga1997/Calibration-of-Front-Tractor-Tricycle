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
clened_DS_from_spaces_path = "clened_dataset_from_spaces.txt"
num_comments_in_DS = 8 # in the given dataset
num_comments_in_clened_DS = 0
tokens_separator = ":"

#[translated_TS_array, FIRST_TS] = translate_TS(DS_path, num_comments, tokens_separator)
#[translated_IncEncInfo_array, FIRST_IncEncInfo] = translate_IncEncInfo(DS_path, num_comments, tokens_separator)

#TS_val = val_TS_translation(DS_path, num_comments, tokens_separator, translated_TS_array, FIRST_TS)
#IncEncInfo_val = val_IncEncInfo_translation(DS_path, num_comments, tokens_separator, translated_IncEncInfo_array, FIRST_IncEncInfo)


clean_dataset_from_unuseful_spaces(DS_path, num_comments_in_DS, clened_DS_from_spaces_path)
data = read_DS(clened_DS_from_spaces_path, num_comments_in_clened_DS, tokens_separator, new_DS_path)


# TS stands for TimeStamp
# DS stands for DataSet
# IncEncInfo stands for Incremental Encoder Information
# num stands for number
# tokens stands for informations

from dataset_validation import *
from timestamp_handler import *
from incremental_encoder_handler import *
from dataset_handler import *
from file_handler import *
import numpy as np
from decimal import Decimal

DS_path = "dataset.txt"
new_DS_path = "new_dataset.txt"
clened_DS_from_spaces_path = "clened_dataset_from_spaces.txt"
num_comments_in_DS = 8 # in the given dataset
num_comments_in_clened_DS = 0
tokens_separator = ":"
unchanged_column_idx = [1,3,4,5,6,7,8]


clean_dataset_from_unuseful_spaces(DS_path, num_comments_in_DS, clened_DS_from_spaces_path)
[outcome, string_data_value, string_new_data_value] = dataset_validation(new_DS_path, clened_DS_from_spaces_path, num_comments_in_clened_DS, tokens_separator, unchanged_column_idx)

if (not outcome):
    print("not daje")
    # [translated_TS_array, FIRST_TS] = reset_TS(DS_path, num_comments_in_DS, tokens_separator)
    # [translated_IncEncInfo_array, FIRST_IncEncInfo] = translate_IncEncInfo_to_avoid_uint64_overflow(DS_path, num_comments_in_DS, tokens_separator)

    # # substitute firts column with translated_TS_array
    # for i in range(0, string_data_value.shape[0]):
    #     string_data_value[i][0] = str(translated_TS_array[i])
    # # substitute third column with translated_TS_array
    # for i in range(0, string_data_value.shape[0]):
    #     string_data_value[i][2] = str(translated_IncEncInfo_array[i])

    # new_DS_file = open(new_DS_path, 'w')
    # for row in range (0, string_data_value.shape[0]):
    #     for column in range(0, string_data_value.shape[1]):
    #         new_DS_file.write(string_data_value[row][column].decode())
    #         if (column != string_data_value.shape[1] - 1):
    #             new_DS_file.write(" ")
    #     new_DS_file.write("\n")

else:
    print("daje")
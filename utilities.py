# num stands for number

def take_one_word_from_line(line, separator, token_idx, word_idx):
    tokens = line.split(separator)
    string = tokens[token_idx].strip()
    splitted_words = string.split(" ")
    word = splitted_words[word_idx]
    return word

def take_one_word_from_file(file_path, line_idx, separator, token_idx, word_idx):
    f = open(file_path)
    lines = f.read().splitlines()
    word = take_one_word_from_line(lines[line_idx], separator, token_idx, word_idx)
    return word

def num_of_lines_in_file(file_path):
    f = open(file_path)
    lines = f.read().splitlines()
    return len(lines)

def num_of_tokens_in_line(file_path, line_idx, separator):
    f = open(file_path)
    lines = f.read().splitlines()
    tokens = lines[line_idx].split(separator)
    return len(tokens)

def num_of_words_in_token(file_path, line_idx, separator, token_idx):
    f = open(file_path)
    lines = f.read().splitlines()
    tokens = lines[line_idx].split(separator)
    string = tokens[token_idx].strip()
    splitted_words = string.split(" ")
    return len(splitted_words)
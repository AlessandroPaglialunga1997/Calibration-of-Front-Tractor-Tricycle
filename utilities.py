def take_one_word_from_line(line, separator, token_index, word_index):
    tokens = line.split(separator)
    string = tokens[token_index].strip()
    splitted_words = string.split(" ")
    word = splitted_words[word_index]
    return word

def take_one_word_from_file(file_path, line_index, separator, token_index, word_index):
    f = open(file_path)
    lines = f.read().splitlines()
    word = take_one_word_from_line(lines[line_index], separator, token_index, word_index)
    return word


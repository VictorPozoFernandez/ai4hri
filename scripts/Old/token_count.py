import nltk
nltk.download('punkt')
from nltk.tokenize import word_tokenize

def count_tokens(text):
    tokens = word_tokenize(text)
    return len(tokens)

text = '''
'''

token_count = count_tokens(text)
print(f"The number of tokens in the text is: {token_count}")
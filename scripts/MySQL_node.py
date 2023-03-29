import rospy
from ai4hri.msg import String_list
import os
import sqlite3
from keybert import KeyBERT
import openai
import re
import ast

DEBUG = rospy.get_param('/MySQL/DEBUG')
kw_model = KeyBERT(model='all-mpnet-base-v2')

# Connect to the Camera_Store database. Initialize the cursor for querying the database.
parent_dir = os.path.dirname(os.path.abspath(__file__))
db_path = os.path.join(parent_dir, "Camera_Store.db")
db2 = sqlite3.connect(db_path, check_same_thread=False)

# Initialize multiple cursors for the database
mycursor = db2.cursor()
mycursor2 = db2.cursor()
mycursor3 = db2.cursor()
mycursor4 = db2.cursor()


def main():

    # Initialize the MySQL ROS node and subscribe to extracted_info topic
    rospy.init_node("MySQL", anonymous=True)
    rospy.loginfo("Node MySQL initialized. Listening...")
    rospy.Subscriber("/ai4hri/extracted_info", String_list, search_callback)

    rospy.spin()


def search_callback(msg):

    # Extract relevant data from the received message
    customer_sentence = msg.data[0]
    shopkeeper_sentence = msg.data[1]
    num_models = int(msg.data[-2])/2
    num_topics = msg.data[-1]
    msg.data.pop(0)
    msg.data.pop(0)
    msg.data.pop(-1)
    msg.data.pop(-1)

    # Extract the camera models and topics of interest
    models_interest = []
    for _ in range(int(num_models)):

        models_interest.append((msg.data[0],msg.data[1]))
        msg.data.pop(0)
        msg.data.pop(0)

    topics_interest = []
    for _ in range(int(num_topics)):

        topics_interest.append(msg.data[0])
        msg.data.pop(0)
    
    # Query the database for the characteristics of the detected topics for each of the presented cameras models.
    mycursor2.execute("SELECT name FROM sqlite_master WHERE type='table'")

    relevant_info = []
    for table_name in mycursor2:

        mycursor3.execute("PRAGMA table_info({})".format(table_name[0]))

        for column_info in mycursor3:

            if column_info[1] in topics_interest:
                for i, model_reference in enumerate(models_interest):
                    relevant_info.append([])

                    mycursor4.execute("SELECT {} FROM {} WHERE Product_ID = ?".format(column_info[1], table_name[0]), (model_reference[0],))

                    for finding in mycursor4:

                        relevant_info[i].append(str(column_info[1]) + ": " + str(finding[0]))

    # Set OpenAI API credentials
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    # Generate message history with system instructions as first prompt. Append customer and shopkeeper utterance. 
    messages_history = generating_system_instructions(models_interest,relevant_info)
    messages_history.append({"role": "user", "content": "Customer utterance: " + str(customer_sentence) + "Shopkeeper utterance: " + str(shopkeeper_sentence)})

    # Get the generated text from OpenAI's GPT-3.5-turbo model
    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo", 
        messages=messages_history,
        temperature=0.0
    )

    #Remove the last message (shopkeeper message) from the message history
    messages_history.pop(-1)

    # Process and print the identified knowledge
    
    substrings = extract_substrings(completion["choices"][0]["message"]["content"])

    for substring in substrings:

        substring = ast.literal_eval(substring)
        feature = get_left_substring(substring[2])
        if feature in topics_interest:
            print("")
            print(substring[0])
            print("Product: " + substring[1])
            print("Feature: " + substring[2])
            print("Reason: " + substring[3])

    if len(substrings) == 0:
        print("")
        print("ChatGPT2: " + str(completion["choices"][0]["message"]["content"]))


def extract_substrings(text):
    pattern = r'##\[(.*?)\]##'
    substrings = re.findall(pattern, text)
    return substrings


def get_left_substring(text):
    if ':' in text:
        left_substring = text.split(':', 1)[0].strip()
        return left_substring
    else:
        return text

def generating_system_instructions(models_interest,relevant_info):

    message1 = """Imagine you are helping me determine if a shopkeeper is right or mistaken when presenting the characteristics of a camera model. Your task is to analyze the shopkeeper's statements and output the relevant lists based on the characteristics mentioned by the shopkeeper:

    1. If the shopkeeper is right about a characteristic, output:  ##['SHOPKEEPER IS RIGHT', '<presented camera model>', '<presented characteristic>', '<Reason of why the shopkeeper is right>']##
    2. If the shopkeeper is mistaken about a characteristic, output:  ##['SHOPKEEPER IS MISTAKEN', '<presented camera model>', '<presented characteristic>', '<Reason of why the shopkeeper is mistaken>']##
    3. If the shopkeeper is not sure about a characteristic, output:  ##['SHOPKEEPER DOESNT KNOW', '<presented camera model>', '<presented characteristic>', '<Reason of why the shopkeeper doesnt know>']##

    Please only consider the camera models and their characteristics provided in the model list. Do not use any hypothetical camera models.

    Model list:
    """

    # Create a list to store characteristics of products
    characteristics_products=[]
    for i, model_reference in  enumerate(models_interest):
        characteristics_products.append(str(model_reference[1]) + ": " + str(relevant_info[i]))

    message2 = """ 

    When the shopkeeper presents multiple characteristics, output a separate list for each characteristic. If the shopkeeper does not mention a specific characteristic, do not output a list about it.

    Here's an example of how to format your answer:

    Customer utterance: <Shopkeeper utterance> Shopkeeper utterance: <Shopkeeper utterance>
    ##['SHOPKEEPER IS RIGHT', '<presented camera model>', '<presented characteristic>', '<Reason of why the shopkeeper is right>']##
    ##['SHOPKEEPER IS MISTAKEN', '<presented camera model>', '<presented characteristic>', '<Reason of why the shopkeeper is mistaken>']##
    ##['SHOPKEEPER DOESNT KNOW', '<presented camera model>', '<presented characteristic>', '<Reason of why the shopkeeper doesnt know>']##

    Always include the ## characters at the beginning and the end of each list. Keep your response concise. 
    """

    # Put together previous string messages to obtain the final prompt that will be sent to ChatGPT.
    system_message = message1 + str(characteristics_products) + message2
    messages_history=[{"role": "system", "content": system_message}]

    return messages_history


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
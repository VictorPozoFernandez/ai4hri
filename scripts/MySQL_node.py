import rospy
from ai4hri.msg import String_list
import os
import sqlite3
import openai
import re
import ast
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage, AIMessage, Document
from langchain.prompts import PromptTemplate
import json


DEBUG = rospy.get_param('/MySQL/DEBUG')

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
    
    ground_truth = extract_ground_truth(models_interest, topics_interest)
    result = judge_gpt(shopkeeper_sentence, ground_truth)
    substrings = extract_substrings(result.content)

    for substring in substrings:

        try:
            substring = ast.literal_eval(substring)
            
        except:
            print("Couldnt ast")
            return
        
        feature = get_left_substring(substring[2])
        model = substring[1]

        if (substring[0] != 'NOT MENTIONED'):
            print("")
            print(substring[0])
            print("Product: " + substring[1])
            print("Feature: " + substring[2])
            print("Reason: " + substring[3])

    if len(substrings) == 0:
        print("")
        print("ChatGPT2: " + str(result.content))


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
    

def extract_ground_truth(models_interest, topics_interest):

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

    # Create a list to store characteristics of products
    characteristics_products=[]
    for i, model_reference in  enumerate(models_interest):
        characteristics_products.append(str(model_reference[1]) + ": " + str(relevant_info[i]))
    return characteristics_products


def judge_gpt(shopkeeper_sentence, ground_truth):

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")

    # Prepare prompt to send, using JSON format
    chat = ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0, openai_api_key=openai_api_key)

    system_prompt = """
    Imagine you are helping me determine if a shopkeeper is right or mistaken when presenting the characteristics of a camera model. Your task is to analyze the shopkeeper's statements and output the relevant lists based on the characteristics mentioned by the shopkeeper:

    1. If the shopkeeper is right about a characteristic, output: ##['SHOPKEEPER IS RIGHT', '<camera model's name>', '<presented characteristic>', '<Reason of why the shopkeeper is right>']##
    2. If the shopkeeper is mistaken about a characteristic, output: ##['SHOPKEEPER IS MISTAKEN', '<camera model's name>', '<presented characteristic>', '<Reason of why the shopkeeper is mistaken>']##
    3. If the shopkeeper does not mention a characteristic, output: ##['NOT MENTIONED', '<camera model's name>', '<presented characteristic>', '<Reason of why the characteristic is not mentioned>']##

    When the shopkeeper presents multiple characteristics, output a separate list for each characteristic.

    Use the information given in Ground Truth to help you. Here's an example of how to format your answer:

    Shopkeeper utterance: <Shopkeeper utterance>
    Ground Truth: ["Sony Alpha a6000: ['Model: Sony Alpha a6000', 'Price: <>', 'Type_of_camera: <>', 'Resolution: <>']"]
    ##['SHOPKEEPER IS RIGHT', 'Sony Alpha a6000', 'Type_of_camera', '<Reason of why the shopkeeper is right>']##
    ##['SHOPKEEPER IS MISTAKEN', 'Sony Alpha a6000', 'Resolution', '<Reason of why the shopkeeper is mistaken>']##
    ##['NOT MENTIONED', '<Sony Alpha a6000', 'Price', '<Reason of why the characteristic is not mentioned>']##
    
    Keep your response concise. 
    Always include the ## characters at the beginning and the end of each list.
    """

    user_template = """
    Shopkeeper: {shopkeeper_sentence}
    Ground Truth: {ground_truth}
    """

    user_prompt_template = PromptTemplate(input_variables=["shopkeeper_sentence", "ground_truth"], template=user_template)
    user_prompt = user_prompt_template.format(shopkeeper_sentence = shopkeeper_sentence, ground_truth = ground_truth)

    prompt_history = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_prompt)
    ]

    result = chat(prompt_history)
    #print(result.content)
    return result


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
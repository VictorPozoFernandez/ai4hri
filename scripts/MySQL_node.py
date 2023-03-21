import rospy
from ai4hri.msg import String_list
import os
import sqlite3
from keybert import KeyBERT
import openai
import ast

DEBUG = rospy.get_param('/MySQL/DEBUG')
kw_model = KeyBERT(model='all-mpnet-base-v2')

# Connect to the Camera_Store database. Initialize the cursor for querying the database.
db2 = sqlite3.connect("/home/victor/catkin_ws/src/ai4hri/scripts/Camera_Store.db", check_same_thread=False)

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
    shopkeeper_sentence = msg.data[0]
    num_models = int(msg.data[-2])/2
    num_topics = msg.data[-1]
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

    # Generate message history with system instructions as first prompt. Append shopkeeper utterance. 
    messages_history = generating_system_instructions(models_interest,relevant_info)
    messages_history.append({"role": "user", "content": "Shopkeeper utterance: " + str(shopkeeper_sentence)})

    # Get the generated text from OpenAI's GPT-3.5-turbo model
    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo", 
        messages=messages_history,
        temperature=0.0
    )

    #Remove the last message (shopkeeper message) from the message history
    messages_history.pop(-1)

    # Process and print the identified knowledge
    try:
        result = ast.literal_eval(completion["choices"][0]["message"]["content"])

        if isinstance((result), list):
            for j in range(len(list(result))):
                print("")
                print(result[j][0])
                print("Product: " + result[j][1])
                print("Feature: " + result[j][2])
                print("Reason: " + result[j][3])
    
    # If the knowledge can't be idetified, print the comment that ChatGPT has generated
    except:
        print("")
        print("ChatGPT: " + str(completion["choices"][0]["message"]["content"]))



def generating_system_instructions(models_interest,relevant_info):

    message1 = """Imagine you are helping me to identify if a shopkeeper is right or mistaken when he presents the characteristics of a camera model. You are required to output the following lists:

    ['SHOPKEEPER IS RIGHT', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is right>]
    ['SHOPKEEPER IS MISTAKEN', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is mistaken>]

    You have the following camera models and their characteristics to choose from. You are not to use any other hypothetical camera models:

    """

    # Create a list to store characteristics of products
    characteristics_products=[]
    for i, model_reference in  enumerate(models_interest):
        characteristics_products.append(str(model_reference[1]) + ": " + str(relevant_info[i]))

    message2 = """ 

    Here is an example that illustrates how can you output your answer:

    Shopkeeper utterance: <Shopkeeper utterance>;
    You: [['SHOPKEEPER IS RIGHT', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is right>], ['SHOPKEEPER IS RIGHT', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is right>],['SHOPKEEPER IS MISTAKEN', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is mistaken>],['SHOPKEEPER IS MISTAKEN', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is mistaken>]]

    Output multiple outputs if you detect that multiple characteristics are presented at the same time. Do not consider the characteristics that don't appear in the Shopkeeper utterance.
    Remember that the shopkeeper utterances are recorded using a microphone and there may be some Automatic Speech Recognition errors. 
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
    
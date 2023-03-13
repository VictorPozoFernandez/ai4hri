import rospy
from ai4hri.msg import String_list
import os
import mysql.connector
from keybert import KeyBERT
import openai
import ast

DEBUG = rospy.get_param('/MySQL/DEBUG')
kw_model = KeyBERT(model='all-mpnet-base-v2')

db = mysql.connector.connect(
  host="localhost",
  user="root",
  password=os.environ.get("MYSQL_PASSWRD"),
  database="Camera_Store")

mycursor = db.cursor()
mycursor2 = db.cursor(buffered=True)
mycursor3 = db.cursor(buffered=True)
mycursor4 = db.cursor(buffered=True)


def main():

    rospy.init_node("MySQL", anonymous=True)
    rospy.loginfo("Node MySQL initialized. Listening...")
    rospy.Subscriber("/ai4hri/extracted_info", String_list, search_callback)

    rospy.spin()


def search_callback(msg):

    shopkeeper_sentence = msg.data[0]
    num_models = int(msg.data[-2])/2
    num_topics = msg.data[-1]
    msg.data.pop(0)
    msg.data.pop(-1)
    msg.data.pop(-1)

    models_interest = []
    for _ in range(int(num_models)):

        models_interest.append((msg.data[0],msg.data[1]))
        msg.data.pop(0)
        msg.data.pop(0)

    topics_interest = []
    for _ in range(int(num_topics)):

        topics_interest.append(msg.data[0])
        msg.data.pop(0)
    
    mycursor2.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE table_schema = 'Camera_Store' AND TABLE_NAME IN (SELECT table_name FROM information_schema.columns WHERE column_name = 'Product_ID')")
 
    relevant_info = []
    for search_row in mycursor2:

        if search_row[0] in topics_interest:
            mycursor3.execute("SELECT table_name FROM information_schema.columns WHERE column_name = %s",search_row)
            
            for row2 in mycursor3:
                
                if row2[0] == 'replication_asynchronous_connection_failover':
                    pass
                
                else:

                    for i, model_reference in enumerate(models_interest):
                        relevant_info.append([])

                        mycursor4.execute("SELECT " + search_row[0] + " FROM " + row2[0] + " WHERE Product_ID = %s", (model_reference[0],))

                        for finding in mycursor4:

                            relevant_info[i].append(str(search_row[0]) + ": " + str(finding[0]))

    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    messages_history = generating_system_instructions(models_interest,relevant_info)
    messages_history.append({"role": "user", "content": "Shopkeeper utterance: " + str(shopkeeper_sentence)})

    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo", 
        messages=messages_history,
        temperature=0.0
    )

    messages_history.pop(-1)

    try:
        result = ast.literal_eval(completion["choices"][0]["message"]["content"])

        if isinstance((result), list):
            for j in range(len(list(result))):
                print("")
                print(result[j][0])
                print("Product: " + result[j][1])
                print("Feature: " + result[j][2])
                print("Reason: " + result[j][3])
    
    except:
        print("")
        print("ChatGPT: " + str(completion["choices"][0]["message"]["content"]))



def generating_system_instructions(models_interest,relevant_info):

    message1 = """Imagine you are helping me to identify if a shopkeeper is right or mistaken when he presents the characteristics of a camera model. You are required to output the following lists:

    ['SHOPKEEPER IS RIGHT', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is right>]
    ['SHOPKEEPER IS MISTAKEN', <presented camera model>, <presented characteristic>, <Reason of why the shopkeeper is mistaken>]

    You have the following camera models and their characteristics to choose from. You are not to use any other hypothetical camera models:

    """

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

    system_message = message1 + str(characteristics_products) + message2
    messages_history=[{"role": "system", "content": system_message}]

    return messages_history


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
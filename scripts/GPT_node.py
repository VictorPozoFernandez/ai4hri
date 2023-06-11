import rospy
from ai4hri.msg import String_list
import openai
import os
import sqlite3
from sklearn.metrics.pairwise import cosine_similarity
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage, AIMessage, Document
from langchain.prompts import PromptTemplate
import json
import re
import ast
import concurrent.futures

global previous_conversations
global current_model
previous_conversations = ""
current_model = ""

DEBUG = rospy.get_param('/GPT/DEBUG')

# Possibility of dynamically changing the products of interest depending on the location of the shop, the type of product that is being discussed (cameras, objectives, etc. )
# In this case the position tracker is not implemented yet, all cameras from Malcom's experiment are considered.
#products_of_interest = [(1,"Nikon Coolpix S2800"),(2,"Sony Alpha a6000"),(3,"Canon EOS 5D Mark III")] 
products_of_interest = [(1,"Nikon Coolpix S2800"),(2,"Sony Alpha a6000"),(3,"Canon EOS 5D Mark III"),(4,"Sony Alpha a5000"),(5,"Canon EOS 1000D")] 

def main():

    # Initialize the GPT ROS node and subscribe to utterance_and_position topic
    rospy.init_node("GPT", anonymous=True)
    rospy.loginfo("Node GPT initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance_and_position", String_list, callback)
    rospy.spin()


def callback(msg):

    global current_model
    global current_topic
        
    # Set OpenAI API credentials
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    # Extract relevant modelS and topic from the message

    with concurrent.futures.ThreadPoolExecutor() as executor:
        future_1 = executor.submit(detect_change_of_camera, msg)
        future_2 = executor.submit(topic_extraction, msg)

        # Retrieve results from futures.
        result = future_1.result()
        topic = future_2.result()

    current_topic = topic

    if (result["Detection"] == "1 camera" and (len(current_model) != 2)) or (result["Detection"] == "2 cameras"):
        characteristics_products = extraction_characteristics_products(products_of_interest, topic)
        detected_model_list= model_identification_gpt(msg, characteristics_products)
        print("Detected model: " + str(detected_model_list))
        current_model = detected_model_list
    
    if ("NULL" not in topic):

        # Initialize the publisher for extracted_info ROS topic
        pub = rospy.Publisher('/ai4hri/extracted_info', String_list, queue_size= 1, latch=True) 

        # Combine models and topic into a single list. Append number of detected models and topics
        extracted_info= String_list()
        extracted_info = current_model + topic  
        extracted_info.append(str(len(current_model)))
        extracted_info.append(str(len(topic)))

        #Insert the first and second element of message data (customer and shopkeeper utterance) at the beginning of the list
        extracted_info.insert(0, msg.data[1])
        extracted_info.insert(0, msg.data[0])

        # Publish the extracted information
        pub.publish(extracted_info)


def detect_change_of_camera(msg):
    
    global previous_conversations
    global current_model
    global current_topic

    if str(msg.data[0]) != "":      
        previous_conversations = previous_conversations + "Customer: " + str(msg.data[0]) + " Shopkeeper: " + str(msg.data[1])
    elif str(msg.data[1]) != "": 
        previous_conversations = previous_conversations + " Shopkeeper: " + str(msg.data[1])

    result = change_of_model_classification_fast(msg)
    
    if result["Detection"] == "1 camera" and current_model != "" and(len(current_model) == 2):
        print("Detected model: " + str(current_model) + (" (They keep talking about the same camera)"))
        
    elif result["Detection"] == "2 cameras":
        previous_conversations = " Shopkeeper: " + str(msg.data[1])

    
    return result


def change_of_model_classification_fast(msg):

    global previous_conversations
    global current_model

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")

    # Prepare prompt to send, using JSON format
    chat = ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0, openai_api_key=openai_api_key)


    system_prompt = """
    You are a helpful assistant that listens a conversation between a Customer and a Shopkeeper inside a camera shop and identifies if different cameras are hinted during the conversation.

    Here is an example that illustrates how can you output your answer.

    Customer: 'Good morning' Shopkeeper: 'How can I help you?';
    You: {"Detection": "0 cameras"}

    Customer: 'How is the price of this camera?' Shopkeeper: 'It has a price of 68 dollars';
    You: {"Detection": "1 camera"}

    Shopkeeper: 'This camera costs 100 dollars' Customer: 'And the price of the first camera that you showed me?' Shopkeeper: '68 dollars';
    You: {"Detection": "2 cameras"}

    Shopkeeper: 'This one is very good' Customer: 'Do you have anything else?' Shopkeeper: 'Yes, this camera over here is available in color red';
    You: {"Detection": "2 cameras"}

    If you detect expressions similar to "and what about this one?" or "do you have anything else", output {"Detection": "2 cameras"}
    Don't output the name of the models. 
    Output the answer only in JSON format.
    """

    user_template = """
    {previous_conversations}
    """

    user_prompt_template = PromptTemplate(input_variables=["previous_conversations"], template=user_template)
    user_prompt = user_prompt_template.format(previous_conversations = previous_conversations)

    prompt_history = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_prompt)
    ]

    if DEBUG == True:
        print("")
        print(user_prompt)
        result=input("change_model_gpt:")
        data = extract_json(result)

    else:
        result = chat(prompt_history)
        data = extract_json(result.content)
    
    return data


def topic_extraction(msg):

    # Connect to the Camera_Store database. Initialize the cursor for querying the database.
    parent_dir = os.path.dirname(os.path.abspath(__file__))
    db_path = os.path.join(parent_dir, "Camera_Store.db")
    db = sqlite3.connect(db_path)
    
    mycursor = db.cursor()

    # Get all table names in the database
    mycursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = mycursor.fetchall()

    # Iterate through tables. Get all column names. Create a list with all the the found column names in the database that can be considered as topic
    column_list = []
    for table in tables:
        table_name = table[0]
        mycursor.execute(f"PRAGMA table_info({table_name});")
        columns = mycursor.fetchall()
        column_names = [column[1] for column in columns]

        if 'Product_ID' in column_names: 
            column_list = column_list + column_names  
        
        column_list_no_duplicates = list(set(column_list))
        column_list = [item for item in column_list_no_duplicates if item != 'Product_ID']

    topics = topic_identification_gpt(msg, column_list)
    topics_list = ast.literal_eval(topics["Detection"])

    print("Detected topics: " + str(topics_list))

    return topics_list


def topic_identification_gpt(msg, column_list):

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")

    # Prepare prompt to send, using JSON format
    chat = ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0, openai_api_key=openai_api_key)


    system_prompt = """
    You are a helpful assistant. Write the element from the List that approximates the most with the topic of conversation given as Input.

    Here there are some examples that illustrates how can you output your answer. The interactions appear in cronological order:

    Input: What is the price of the Sony Camera? It has a price of 550 dollars
    List: ['Type_of_camera', 'Model', 'Price', 'ISO', 'Camera_features', 'Color', 'Weight', 'Resolution']
    You: {"Detection": "['Price']"}

    Input: And what about this one? This model is available in color blue and has a resolution of 20 megapixels
    List: ['Type_of_camera', 'Model', 'Price', 'ISO', 'Camera_features', 'Color', 'Weight', 'Resolution']
    You: {"Detection": "['Color', 'Resolution']"}

    Input: Good morning, how can I be of service?
    List: ['Type_of_camera', 'Model', 'Price', 'ISO', 'Camera_features', 'Color', 'Weight', 'Resolution']
    You: {"Detection": "['NULL']"}

    Output the answer only in JSON format.  If no topic is detected, output {"Detection": "['NULL']"}
    Output only topics that appear in the List.
    """

    user_template = """
    Input: {customer}. {shopkeeper}
    List: {column_list}
    """

    user_prompt_template = PromptTemplate(input_variables=["customer", "shopkeeper", "column_list"], template=user_template)
    user_prompt = user_prompt_template.format(customer = msg.data[0], shopkeeper = msg.data[1], column_list = column_list)

    prompt_history = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_prompt)
    ]


    if DEBUG == True:
        print("")
        print(user_prompt)
        result=input("topic_identification_gpt:")
        data = extract_json(result)
        
    result = chat(prompt_history)
    data = extract_json(result.content)

    return data


def extract_json(s):
    json_match = re.search(r'\{.*\}', s, re.DOTALL)
    if json_match:
        try:
            data = json.loads(json_match.group())
            return data
        except json.JSONDecodeError:
            print("Invalid JSON")
            return None
    else:
        print("No JSON found in the string")
        return {"Detection": "null", "Model": "null", "Output" : "null",}


def extraction_characteristics_products(products_of_interest, topics):

    # Connect to the Camera_Store database. Initialize the cursor for querying the database.
    parent_dir = os.path.dirname(os.path.abspath(__file__))
    db_path = os.path.join(parent_dir, "Camera_Store.db")
    db = sqlite3.connect(db_path)
    
    mycursorGPT = db.cursor()
    mycursorGPT2 = db.cursor()
    mycursorGPT3 = db.cursor()

    # List to store extracted characteristics for each product
    characteristics_product_IDs = []

    for Product in products_of_interest:

        # List to store characteristics for the current product
        characterstics_model = []

        # Get table names that have a 'Product_ID' column
        mycursorGPT.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = mycursorGPT.fetchall()

        for table in tables:
            table_name = table[0]
            mycursorGPT2.execute(f"PRAGMA table_info({table_name});")
            columns = mycursorGPT2.fetchall()
            column_names = [column[1] for column in columns]

            # Check if 'Product_ID' exists in the table
            if 'Product_ID' in column_names:
                for column_name in column_names:
                    if column_name in topics or column_name == "Model":
                        # Get the value of the current column for the current product
                        mycursorGPT3.execute(f"SELECT {column_name} FROM {table_name} WHERE Product_ID = ?", (Product[0],))
                        column = []

                        for characteristic in mycursorGPT3:

                            # Add the characteristic value to the column list
                            column.append(characteristic[0])

                        # Append the column name and its values as a tuple to the characterstics_model list
                        characterstics_model.append((column_name, column))

        # Append the characteristics for the current product to the main list
        characterstics_model.sort(key=lambda x: x[0] != 'Model')
        characteristics_product_IDs.append(characterstics_model)

    # Return the list containing extracted characteristics for each product
    return characteristics_product_IDs



def model_identification_gpt(msg, characteristics_products):

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")

    # Prepare prompt to send, using JSON format
    chat = ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0, openai_api_key=openai_api_key)


    system_prompt = """
    You are a helpful assistant. Identify the camera model from the List that is currently being discussed 

    Here there are some examples that illustrates how can you output your answer.

    'What is the price of this Nikon Camera? it costs 68 dollars only';
    List: [[('Model', ['Nikon Coolpix S2800']), ('Price', ['68'])], [('Model', ['Sony Alpha a6000']), ('Price', ['550'])], [('Model', ['Canon EOS 5D Mark III']), ('Price', ['2000'])]]
    You: {"Detection": "['Nikon Coolpix S2800']"}

    'And what about this camera?'this model is only available in color black';
    List: [[('Model', ['Nikon Coolpix S2800']), ('Color', ['black', 'pink', 'purple', 'red', 'silver'])], [('Model', ['Sony Alpha a6000']), ('Color', ['black', 'silver', 'white'])], [('Model', ['Canon EOS 5D Mark III']), ('Color', ['black'])]]
    You: {"Detection": "['Canon EOS 5D Mark III']"}

    'Has this camera 18 preset modes? No, it only has 9 preset modes';
    List: [[('Model', ['Nikon Coolpix S2800']), ('Camera_features', ['12 Glamour retouch effects', '18 preset modes', 'Optical Zoom 5x'])], [('Model', ['Sony Alpha a6000']), ('Camera_features', ['13 artistic effect modes', '9 preset modes'])], [('Model', ['Canon EOS 5D Mark III']), ('Camera_features', ['Silent Shooting'])]]
    You: {"Detection": "['Sony Alpha a6000']"}

    'And the weight of this one? This camera weights 95 grams';
    List: [[('Model', ['Nikon Coolpix S2800']), ('Weight', ['120 grams'])], [('Model', ['Sony Alpha a6000']), ('Weight', ['470 grams'])], [('Model', ['Canon EOS 5D Mark III']), ('Weight', ['950 grams'])]]
    You: {"Detection": "['NULL']"}

    'How much does it cost this camera? This one costs 200 dollars';
    List: [[('Model', ['Nikon Coolpix S2800']), ('Price', ['68'])], [('Model', ['Sony Alpha a6000']), ('Price', ['550'])], [('Model', ['Canon EOS 5D Mark III']), ('Price', ['2000'])]]
    You: {"Detection": "['NULL']"}

    'What about its resolution? It has more than 21 megapixels';
    List: [[('Model', ['Nikon Coolpix S2800']), ('Resolution', ['20.1 megapixels'])], [('Model', ['Sony Alpha a6000']), ('Resolution', ['24.0 megapixels'])], [('Model', ['Canon EOS 5D Mark III']), ('Resolution', ['22.3 megapixels'])]]
    You: {"Detection": "['Sony Alpha a6000', 'Canon EOS 5D Mark III']"}

    Use the data from the List to identify the camera model. If no camera is detected, output {"Detection": "['NULL']"}. If two or more models are detected, output all the detected models . 
    Output the answer only in JSON format.
    """

    user_template = """
    {customer}. {shopkeeper};
    List: {characteristics_products};
    """

    user_prompt_template = PromptTemplate(input_variables=["customer", "shopkeeper", "characteristics_products"], template=user_template)
    user_prompt = user_prompt_template.format(customer = msg.data[0], shopkeeper = msg.data[1], characteristics_products = characteristics_products)

    prompt_history = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_prompt)
    ]

    if DEBUG == True:
        print("")
        print(user_prompt)
        result=input("model_identification_gpt:")
        data = extract_json(result)

    else:
        result = chat(prompt_history)
        data = extract_json(result.content)

    detected_model_list = []
    # Iterate through products_of_interest and check if they are mentioned in the generated text from ChatGPT
    
    for Product in products_of_interest:

        if Product[1] in data["Detection"]:
            detected_model_list.append(str(Product[0]))
            detected_model_list.append(str(Product[1]))
    
    if len(detected_model_list) == 0:
        detected_model_list.append(str("NULL"))

    return detected_model_list



if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    





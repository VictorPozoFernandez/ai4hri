import rospy
from ai4hri.msg import String_list
import os
import sqlite3
import re
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage
from langchain.prompts import PromptTemplate
import json

global last_insert_rowid
last_insert_rowid = ""

GPT4 = rospy.get_param('/MySQL/GPT4')
UPDATE = rospy.get_param('/MySQL/UPDATE')

# Connect to the Camera_Store database. Initialize the cursor for querying the database.
script_dir = os.path.dirname(os.path.abspath(__file__))  # Returns the absolute path to the script
parent_dir = os.path.dirname(script_dir)
file_path = os.path.join(parent_dir, 'Database/Camera_Store.db')
db2 = sqlite3.connect(file_path, check_same_thread=False)

# Initialize multiple cursors for the database
mycursor = db2.cursor()
mycursor2 = db2.cursor()
mycursor3 = db2.cursor()
mycursor4 = db2.cursor()
mycursor5 = db2.cursor()
mycursor6 = db2.cursor()
mycursor7 = db2.cursor()
mycursor8 = db2.cursor()


def main():

    # Initialize the MySQL ROS node and subscribe to extracted_info topic
    rospy.init_node("MySQL", anonymous=True)
    rospy.loginfo("Node MySQL initialized. Listening...")
    rospy.Subscriber("/ai4hri/extracted_info", String_list, search_callback)

    rospy.spin()


def search_callback(msg):

    global last_insert_rowid

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
    detections = judge_gpt(shopkeeper_sentence, ground_truth)
    
    if last_insert_rowid == "":

        if UPDATE != "":
            last_insert_rowid = UPDATE
            mycursor5.execute("DELETE FROM Detections WHERE Interaction_ID=?", (int(last_insert_rowid),))
        
        else:
            mycursor5.execute("SELECT MAX(Interaction_ID) FROM Detections;")

            for result in mycursor5:

                if result[0] != None:
                    last_insert_rowid = result[0] + 1
                else:
                    last_insert_rowid = 1

    try:
        
        for detection in detections:

            if (detection["Detection"] != 'NOT MENTIONED') and (detection["Feature"] in topics_interest):
                
                print("")
                if detection["Detection"] == "SHOPKEEPER IS RIGHT":
                    print("\033[92m" + detection["Detection"] + "\033[0m")
                elif detection["Detection"] == "SHOPKEEPER IS MISTAKEN":
                    print("\033[91m" + detection["Detection"] + "\033[0m")
                else:
                    print("\033[93m" + detection["Detection"] + "\033[0m")

                #print("Product: " + substring[2])
                #print("Feature: " + feature)
                print("Reason: " + detection["Explanation"])

                query = "INSERT INTO Detections (Interaction_ID, Utterance, Class, Feature, Reason) VALUES (?, ?, ?, ?, ?);"
                params = (last_insert_rowid ,customer_sentence + " // " + shopkeeper_sentence, detection["Detection"], detection["Feature"], detection["Explanation"])
                mycursor7.execute(query, params)

                db2.commit()

    except Exception as e:
        print(e)
    

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
    if GPT4 == True:
        chat = ChatOpenAI(model_name="gpt-4", temperature=0, openai_api_key=openai_api_key)
    else:
        chat = ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0, openai_api_key=openai_api_key)

    system_prompt = """
    You are a helpful assistant that identifies if a shopkeeper is mistaken when presenting the characteristics of a 
    camera model. Your task is to analyze the information given in Ground Truth and output the relevant JSON objects
    based on the characteristics mentioned by the shopkeeper:

    1. If the shopkeeper is right about a characteristic from Ground Truth, output:
    [
        {
            "Explanation": "<Explain why the Shopkeeper is right>",
            "Detection": "SHOPKEEPER IS RIGHT",
            "Model": "<presented camera model>",
            "Feature": "<feature from Ground Truth>"
        }
    ]

    2. If the shopkeeper is mistaken about a characteristic from Ground Truth, output: 
    [
        {
            "Explanation": "<Explain why the Shopkeeper is mistaken>",
            "Detection": "SHOPKEEPER IS MISTAKEN",
            "Model": "<presented camera model>",
            "Feature": "<feature from Ground Truth>"
        }
    ]

    3. If the shopkeeper does not mention a characteristic from Ground Truth, output: 
    [
        {
            "Explanation": "null",
            "Detection": "NOT MENTIONED",
            "Model": "<presented camera model>",
            "Feature": "<feature from Ground Truth>"
        }
    ]

    4. If the shopkeeper is not able to answer, or asks for help, output:
    [
        {
            "Explanation": "<Explain why the Shopkeeper is not able to answer",
            "Detection": "SHOPKEEPER DOESNT KNOW",
            "Model": "<presented camera model>",
            "Feature": "<feature from Ground Truth>"
        }
    ]
    
    When the shopkeeper presents multiple characteristics, output an JSON object for each characteristic mentioned 
    by the shopkeeper.  Use the information given in Ground Truth to help you reason. 
    
    Here's an example of how to format your answer:

    Shopkeeper utterance: The Sony camera is a Mirrorless camera that has a resolution of 20 megapixels
    Ground Truth: ["Sony Alpha a6000: ['Model: Sony Alpha a6000', 'Price: <550>', 'Type_of_camera: <Mirrorless>', 'Resolution: <18.0 megapixels>']"]
    You:[
            {
                "Explanation": "The Sony Alpha a6000 is indeed a Mirrorless camera",
                "Detection": "SHOPKEEPER IS RIGHT",
                "Model": "Sony Alpha a6000",
                "Feature": "Type_of_camera"
            },
            {
                "Explanation": "The Sony Alpha a6000 has a resolution of 18 megapixels, not 20",
                "Detection": "SHOPKEEPER IS MISTAKEN",
                "Model": "Sony Alpha a6000",
                "Feature": "Resolution"
            },
            {
                "Explanation": "The Shopkeeper doesnt mention any price",
                "Detection": "NOT MENTIONED",
                "Model": "Sony Alpha a6000",
                "Feature": "Price"
            }
        ]

    Keep your response concise. 
    Output the answer only in JSON format.
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
    data = extract_json(result.content)

    return data


def extract_json(s):
    json_match = re.search(r'\[.*\]', s, re.DOTALL)
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


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    

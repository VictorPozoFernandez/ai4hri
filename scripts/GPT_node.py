import rospy
from ai4hri.msg import String_list
import openai
import os
import sqlite3
from sklearn.metrics.pairwise import cosine_similarity

DEBUG = rospy.get_param('/GPT/DEBUG')

# Possibility of dynamically changing the products of interest depending on the location of the shop, the type of product that is being discussed (cameras, objectives, etc. )
# In this case the position tracker is not implemented yet, all cameras from Malcom's experiment are considered.
products_of_interest = [(1,"Nikon Coolpix S2800"),(2,"Sony Alpha a6000"),(3,"Canon EOS 5D Mark III")] 


def main():

    # Initialize the GPT ROS node and subscribe to utterance_and_position topic
    rospy.init_node("GPT", anonymous=True)
    rospy.loginfo("Node GPT initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance_and_position", String_list, callback)
    rospy.spin()


def callback(msg):
        
    # Set OpenAI API credentials
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    # Extract relevant modelS and topic from the message
    detected_model_list = models_of_interest(msg)

    if "NULL" not in detected_model_list:
        topic = topic_extraction(msg)

        # Initialize the publisher for extracted_info ROS topic
        pub = rospy.Publisher('/ai4hri/extracted_info', String_list, queue_size= 1, latch=True) 

        # Combine models and topic into a single list. Append number of detected models and topics
        extracted_info= String_list()
        extracted_info = detected_model_list + topic  
        extracted_info.append(str(len(detected_model_list)))
        extracted_info.append(str(len(topic)))

        #Insert the second element of message data (shopkeeper utterance) at the beginning of the list
        extracted_info.insert(0, msg.data[1])

        # Publish the extracted information
        pub.publish(extracted_info)
    
    else: 
        print("Unrecognized model")  

def models_of_interest(msg):

    # Append the current interaction between the customer and shopkeeper to messages_history
    messages_history.append({"role": "user", "content": "CURRENT INTERACTION: " + msg.data[0] + " ### " + msg.data[1]})

    # Remove the oldest interaction in messages_history if it contains more that 10 interactions (used to define the long-term memory of Chat-GPT in a given context)
    if len(messages_history) > 10:
        messages_history.pop(1)

    # Get the generated text from OpenAI's GPT-3.5-turbo model
    completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo", 
    messages=messages_history,
    temperature=0.0)

    if DEBUG == True:
        print("")
        print("ChatGPT1: " + str(completion["choices"][0]["message"]["content"]))
    
    detected_model_list = []
    # Iterate through products_of_interest and check if they are mentioned in the generated text from ChatGPT
    
    for Product in products_of_interest:

        if Product[1] in completion["choices"][0]["message"]["content"]:
            detected_model_list.append(str(Product[0]))
            detected_model_list.append(str(Product[1]))
    
    if len(detected_model_list) == 0:
        detected_model_list.append(str("NULL"))

    # Update messages_history with the previous interaction
    messages_history.pop(-1)
    messages_history.append({"role": "user", "content": "PREVIOUS INTERACTION: " + msg.data[0] + " ### " + msg.data[1]})
    
    return detected_model_list


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
    utterances_to_compare = []
    for table in tables:
        table_name = table[0]
        mycursor.execute(f"PRAGMA table_info({table_name});")
        columns = mycursor.fetchall()
        column_names = [column[1] for column in columns]

        if 'Product_ID' in column_names:
            utterances_to_compare = utterances_to_compare + column_names  

    # Insert the current interaction between the customer and shopkeeper at the beginning of the list (it will be used to compare it with the previous column names)
    interaction = msg.data[0] + msg.data[1]
    utterances_to_compare.insert(0, interaction)

    # Define the text embedding model and create the corresponding embeddings from the stored utterance and colum names
    model = "text-embedding-ada-002"
    res = openai.Embedding.create( input = utterances_to_compare, engine=model)
    embedded_columns=[]
    for vec in res["data"]:
        embedded_columns.append(vec["embedding"])

    # Separate the utterance embedding from the column embeddings
    utterance = embedded_columns[0]
    embedded_columns.pop(0)
    utterances_to_compare.pop(0)

    # Calculate cosine similarity scores between the utterance and column embedding
    scores = []
    for _, column_candidate in enumerate(embedded_columns):
        score = cosine_similarity([utterance],[column_candidate])
        scores.append(score)

    # Select the top 3 scoring columns
    selected_columns = []
    best_scores = sorted(zip(scores, utterances_to_compare), reverse=True)[:4]

    for score in best_scores:
        selected_column= score[1].replace(" ", "_")
        selected_columns.append(selected_column)

    if DEBUG == True:
        print("Topics: " + str(selected_columns))

    return selected_columns


def generating_system_instructions(products_of_interest):

    message1 = """Imagine you are helping me to identify the camera model that a shopkeeper is presenting to a customer. You are required to output the following answers:

    - Camera model: All possible camera models from the Authorized list that the shopkeeper is presenting in the current interaction. Remember to use the previous interactions to have more context about all the possible camera models that the shopkeeper is presenting in the CURRENT INTERACTION

    Authorized list:
    """

    # Extract all the characteristics of the selected products from the SQL Database. 
    # It will be used by ChatGPT to reason which product is the shopkeeper presenting, even if the name of the model is not explicitly said. 
    characteristics_product_IDs = extraction_characteristics_products(products_of_interest)

    message2 = """ Here is an example that illustrates how can you output your answer. The interactions appear in cronological order:

    Previous interaction: <Customer sentence> ### <Shopkeeper sentence>;
    Current interaction <Customer sentence> ### <Shopkeeper sentence>;
    You: Camera model - <all possible authorized camera model names from the Authorized list> 

    Output only the camera model name that the shopkeeper is presenting in the current interaction. Output only the model's name. Be concise.
    """

    # Put together previous string messages to obtain the final prompt that will be sent to ChatGPT.
    system_message = message1 + str(characteristics_product_IDs) + message2
    
    if DEBUG == True:
        print("")
        print("Prompt sent to ChatGPT:")
        print("")
        print(system_message)

    return system_message


def extraction_characteristics_products(products_of_interest):

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
                    if column_name != 'Product_ID':
                        # Get the value of the current column for the current product
                        mycursorGPT3.execute(f"SELECT {column_name} FROM {table_name} WHERE Product_ID = ?", (Product[0],))
                        column = []

                        for characteristic in mycursorGPT3:

                            # Add the characteristic value to the column list
                            column.append(characteristic[0])

                        # Append the column name and its values as a tuple to the characterstics_model list
                        characterstics_model.append((column_name, column))

        # Append the characteristics for the current product to the main list
        characteristics_product_IDs.append(characterstics_model)

    # Return the list containing extracted characteristics for each product
    return characteristics_product_IDs


if __name__ == '__main__':

    try:
        system_message = generating_system_instructions(products_of_interest)
        messages_history=[{"role": "system", "content": system_message}]
        main()
    
    except rospy.ROSInterruptException:
        pass
    





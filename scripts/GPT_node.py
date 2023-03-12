import rospy
from ai4hri.msg import String_list
import openai
import os
import mysql.connector
from keybert import KeyBERT
from sklearn.metrics.pairwise import cosine_similarity

DEBUG = rospy.get_param('/GPT/DEBUG')

products_of_interest = [(1,"Nikon Coolpix S2800"),(2,"Sony Alpha a6000"),(3,"Canon EOS 5D Mark III")] 
# Possibility of dynamically changing the products of interest depending on the location of the shop, the type of product that is being discussed (cameras, objectives, etc. )


def main():

    rospy.init_node("GPT", anonymous=True)
    rospy.loginfo("Node GPT initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance_and_position", String_list, callback)

    rospy.spin()


def callback(msg):
        
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    detected_model_list = cameras_of_interest(msg)
    keywords = keyword_extraction(msg)
    topic = topic_extraction(msg)

    pub = rospy.Publisher('/ai4hri/extracted_info', String_list, queue_size= 1) 

    extracted_info= String_list()
    extracted_info = detected_model_list + topic + keywords  
    extracted_info.append(str(len(detected_model_list)))
    extracted_info.append(str(len(topic)))

    pub.publish(extracted_info)


def cameras_of_interest(msg):

    messages_history.append({"role": "user", "content": "CURRENT INTERACTION: " + msg.data[0] + " ### " + msg.data[1]})

    if len(messages_history) > 4:
        messages_history.pop(1)

    completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo", 
    messages=messages_history,
    temperature=0.0
    )

    if DEBUG == True:
        print("")
        print(completion["choices"][0]["message"]["content"])
    
    detected_model_list = []
    for Product in products_of_interest:

        if Product[1] in completion["choices"][0]["message"]["content"]:
            detected_model_list.append(str(Product[0]))
            detected_model_list.append(str(Product[1]))

    messages_history.pop(-1)
    messages_history.append({"role": "user", "content": "PREVIOUS INTERACTION: " + msg.data[0] + " ### " + msg.data[1]})
    
    return detected_model_list


def keyword_extraction(msg):

    kw_model = KeyBERT(model='all-mpnet-base-v2')
    keywords = kw_model.extract_keywords(msg.data[1], keyphrase_ngram_range=(1,1), use_maxsum=False, top_n=10)

    keyword_list =[]
    for keyword in keywords:
            
            keyword_list.append(str(keyword[0]))

    if DEBUG == True:
        print("Keywords: " + str(keyword_list))

    return keyword_list


def topic_extraction(msg):

    db = mysql.connector.connect(
    host="localhost",
    user="root",
    password=os.environ.get("MYSQL_PASSWRD"),
    database="Camera_Store"
    )

    mycursor = db.cursor()
    mycursor.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE table_schema = 'Camera_Store' AND TABLE_NAME IN (SELECT table_name FROM information_schema.columns WHERE column_name = 'Product_ID')")

    utterances_to_compare = []
    for row in mycursor:

        utterances_to_compare.append(row[0])

    interaction = msg.data[0] + msg.data[1]
    utterances_to_compare.insert(0, interaction)

    model = "text-embedding-ada-002"
    res = openai.Embedding.create( input = utterances_to_compare, engine=model)

    embedded_columns=[]
    for vec in res["data"]:

        embedded_columns.append(vec["embedding"])

    utterance = embedded_columns[0]
    embedded_columns.pop(0)
    utterances_to_compare.pop(0)

    scores = []
    for _, column_candidate in enumerate(embedded_columns):

        score = cosine_similarity([utterance],[column_candidate])
        scores.append(score)

    selected_columns = []
    best_scores = sorted(zip(scores, utterances_to_compare), reverse=True)[:3]

    for score in best_scores:

        selected_column= score[1].replace(" ", "_")
        selected_columns.append(selected_column)

    if DEBUG == True:
        print("Topics: " + str(selected_columns))

    return selected_columns


def generating_system_instructions(products_of_interest):

    message1 = """Imagine you are helping me to identify the camera model that a shopkeeper is presenting to a customer. You are required to output the following answers:

    - CAMERA MODEL: All possible camera models that the shopkeeper is presenting in the CURRENT INTERACTION.

    You have the following camera models to choose from. You are not to use any other hypothetical camera models:

    """

    characteristics_product_IDs = extraction_characteristics_products(products_of_interest)

    message2 = """ 

    Here is an example that illustrates how can you output your answer. The PREVIOUS INTERACTIONS appear in cronological order:

    PREVIOUS INTERACTION: <Customer sentence> ### <Shopkeeper sentence>;
    CURRENT INTERACTION: <Customer sentence> ### <Shopkeeper sentence>;
    You: CAMERA MODEL - <all possible camera model names> 

    Remember to use the PREVIOUS INTERACTIONS to have more context about all the possible camera models that the shopkeeper is presenting in the CURRENT INTERACTION
    Output only the camera model name that the shopkeeper is presenting in the CURRENT INTERACTION 
    Aproximate the characteristics of the camera that appear in <Shopkeeper sentence> with the characteristics of the camera models to identify them.
    """

    system_message = message1 + str(characteristics_product_IDs) + message2
    
    if DEBUG == True:
        print("")
        print("Prompt sent to ChatGPT:")
        print("")
        print(system_message)

    return system_message


def extraction_characteristics_products(products_of_interest):

    db = mysql.connector.connect(
    host="localhost",
    user="root",
    password=os.environ.get("MYSQL_PASSWRD"),
    database="Camera_Store")

    mycursorGPT = db.cursor(buffered=True)
    mycursorGPT2 = db.cursor(buffered=True)
    mycursorGPT3 = db.cursor(buffered=True)

    characteristics_product_IDs = []
    for Product in products_of_interest:

        characterstics_model = []
        mycursorGPT.execute("SELECT table_name FROM information_schema.columns WHERE column_name = 'Product_ID'")

        for table_name in mycursorGPT:

            mycursorGPT2.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE TABLE_NAME = '" + table_name[0] + "'")
            
            for column_name in mycursorGPT2:
            
                if column_name[0] != 'Product_ID':
                    mycursorGPT3.execute("SELECT " + column_name[0]+ " FROM " + table_name[0] + " WHERE Product_ID = " + str(Product[0]))
                    column = []

                    for characteristic in mycursorGPT3:

                        column.append(characteristic[0])
                    
                    characterstics_model.append((column_name[0], column))
                
        characteristics_product_IDs.append(characterstics_model)

    return characteristics_product_IDs


if __name__ == '__main__':

    try:
        system_message = generating_system_instructions(products_of_interest)
        messages_history=[{"role": "system", "content": system_message}]
        main()
    
    except rospy.ROSInterruptException:
        pass
    





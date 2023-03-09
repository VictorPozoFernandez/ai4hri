import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
from ai4hri.msg import String_list_list
import os
import numpy as np
import mysql.connector
from collections import Counter
import random
from keybert import KeyBERT
import re
import openai
from sklearn.metrics.pairwise import cosine_similarity
#DEBUG = rospy.get_param('/whisper/DEBUG')

openai.organization = os.environ.get("OPENAI_ORG_ID")
openai.api_key = os.environ.get("OPENAI_API_KEY")
model = "text-embedding-ada-002"

kw_model = KeyBERT(model='all-mpnet-base-v2')

db = mysql.connector.connect(
  host="localhost",
  user="root",
  password=os.environ.get("MYSQL_PASSWRD"),
  database="Camera_Store"
)
mycursor = db.cursor()
mycursor2 = db.cursor(buffered=True)
mycursor3 = db.cursor(buffered=True)
mycursor4 = db.cursor(buffered=True)


def main():

    rospy.init_node("MySQL", anonymous=True)
    rospy.loginfo("Node MySQL initialized. Listening...")
    rospy.Subscriber("/ai4hri/keywords", String_list, callback)
    rospy.Subscriber("/ai4hri/extracted_info", String_list, search_callback)


    rospy.spin()

def callback(msg):

    global keywords
    global camera_reference

    mycursor.execute("SELECT Product_ID,Model FROM Camera")

    models = []
    for row in mycursor:
        models.append(row)

    possible_models = []
    possible_models_2 = []    
    
    keywords = []

    for keyword in msg.data:

        if keyword not in keywords:
            keywords.append(keyword)

        for row in models:

            if keyword in row[1].lower():
                possible_models.append(row)

    if len(possible_models)!=0:

        counts = Counter(possible_models)
        max_count = counts.most_common(1)[0][1]

        for value, count in counts.most_common():
            if count == max_count:
                possible_models_2.append(value)
        
        if len(possible_models_2) == 1:
            camera_reference = possible_models_2[0] 
        
        else:
            #Look the costumer and shopkeeper position to identify the camera they are refering to. For now, we choose random from the possible_models_2 list
            camera_reference = random.choice(possible_models_2)


    else:
        #Look the costumer and shopkeeper position to identify the camera they are refering to. For now, we choose random from the models list
        camera_reference = random.choice(models)

    # if camera_reference =! camera closest to costumer and shopkeeper position:
            #camera_reference_2 = (Look the costumer and shopkeeper position to identify the camera they are using as comparison)

def search_callback(msg):

    num_cameras = msg.data[-2]
    num_topics = msg.data[-1]
    msg.data.pop(-1)
    msg.data.pop(-1)

    cameras_interest = []
    for i in range(int(num_cameras)):
        cameras_interest.append(msg.data[0])
        msg.data.pop(0)

    topics_interest = []
    for i in range(int(num_topics)):
        topics_interest.append(msg.data[0])
        msg.data.pop(0)

    keywords_interest = msg.data

    print("Cameras of interest: " + str(cameras_interest))
    print("Topics of interest: " + str(topics_interest))
    print("keywords of interest: " + str(keywords_interest))
    
    mycursor2.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE table_schema = 'Camera_Store' AND TABLE_NAME IN (SELECT table_name FROM information_schema.columns WHERE column_name = 'Product_ID')")
 
    for search_row in mycursor2:

        if search_row[0] in topics_interest:
            mycursor3.execute("SELECT table_name FROM information_schema.columns WHERE column_name = %s",search_row)
            
            for row2 in mycursor3:
                
                if row2[0] == 'replication_asynchronous_connection_failover':
                    pass
                
                else:

                    #if DEBUG == True: 
                    print("Searching column '" + search_row[0] + "' in table '" + str(row2[0])+"':")

                    for camera_reference in cameras_interest:

                        mycursor4.execute("SELECT " + search_row[0] + " FROM " + row2[0] + " WHERE Product_ID = %s", (camera_reference,))

                        for finding in mycursor4:

                            finding_keywords = kw_model.extract_keywords(str(finding[0]), keyphrase_ngram_range=(1,1), use_maxsum=False, top_n=10)
                            count = 0
                            digit_condition = False
                            condition_already_met = False

                            for keyword in keywords_interest:

                                if keyword.isdigit():
                                    digit_condition = True  
                                    
                                    if keyword.lower() in str(finding[0]).lower():   
                                        digit_condition = False 
                                        condition_already_met = True  

                                if keyword.lower() in str(finding[0]).lower(): 
                                    count +=1
                            
                            if ((digit_condition == False) or (condition_already_met==True)) and ((count/len(finding_keywords))>0.60):
                                print("      - The shopkeeper knows that the " + str(camera_reference) + " camera has " + str(finding[0]) + " as " + str(search_row[0]) + " property")
    
                         


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
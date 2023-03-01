import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import os
import numpy as np
import mysql.connector
from collections import Counter
import random
from keybert import KeyBERT
import re
import openai
from sklearn.metrics.pairwise import cosine_similarity
DEBUG = rospy.get_param('/whisper/DEBUG')

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
    rospy.Subscriber("/ai4hri/topics", String_list, search_callback)


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

    if DEBUG == True: 
        print("Camera of reference: " + camera_reference[1]) #See if there is a problem with paralel·l execution of ros nodes. If there is, execute nodes in a streamline way (see notes photo)
        print(" ")
        
    last_utterance = msg.data[0]
    last_utterance = last_utterance.split()
    msg.data.pop(0)

    mycursor2.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE table_schema = 'Camera_Store' AND TABLE_NAME IN (SELECT table_name FROM information_schema.columns WHERE column_name = 'Product_ID')")
    
    for search_row in mycursor2:

        if search_row[0] in msg.data:
            mycursor3.execute("SELECT table_name FROM information_schema.columns WHERE column_name = %s",search_row)
            
            for row2 in mycursor3:

                if row2[0] == 'replication_asynchronous_connection_failover':
                    pass
                
                else:

                    if DEBUG == True: 
                        print("Searching column '" + search_row[0] + "' in table '" + str(row2[0])+"':")

                    mycursor4.execute("SELECT " + search_row[0] + " FROM " + row2[0] + " WHERE Product_ID = %s", (camera_reference[0],))

                    for finding in mycursor4:

                        finding_keywords = kw_model.extract_keywords(str(finding[0]), keyphrase_ngram_range=(1,1), use_maxsum=False, top_n=10)
                        count = 0
                        digit_condition = False
                        candidates_digit = []
                        candidates_bag = []

                        for keyword in keywords:

                            if keyword.isdigit():
                            
                                try:   
                                    pos = last_utterance.index(keyword)
                                    bag_of_words = []
                                    try:
                                        bag_of_words = last_utterance[pos-1] + " " + last_utterance[pos+1]
                                    except:
                                        try:
                                            bag_of_words = last_utterance[pos+1]
                                        except:
                                            try:
                                               bag_of_words = last_utterance[pos-1]
                                            except:
                                                bag_of_words = last_utterance[pos]

                                    candidates_digit.append(keyword)
                                    candidates_bag.append(bag_of_words)
                                    

                                    if keyword.lower() in str(finding[0]).lower(): 
                                        count +=1
                                        selected_digit = keyword   
                                        digit_condition = True   
                                            
                                
                                except:
                                    pass
                                
                            else:

                                if keyword.lower() in str(finding[0]).lower(): 
                                    count +=1

                        if digit_condition == True:
                            candidates_bag.insert(0,str(finding[0]))

                            res = openai.Embedding.create( input = candidates_bag, engine=model)

                            candidates_embedded = []
                            scores = []
                            for vec in res["data"]:
                                candidates_embedded.append(vec["embedding"])
                            finding_embedded = candidates_embedded[0]
                            candidates_embedded.pop(0)
                            candidates_bag.pop(0)

                            for candidate_embedded in candidates_embedded:
                                score = cosine_similarity([finding_embedded],[candidate_embedded])
                                scores.append(score)

                            best_score = sorted(zip(scores, candidates_digit), reverse=True)[:1]
                            

                            if best_score[0][1] == selected_digit:
                                digit_condition = False
                        
                        if (digit_condition == False) and ((count/len(finding_keywords))>0.60): 
                            print("      - The shopkeeper knows that the " + str(camera_reference[1]) + " camera has " + str(finding[0]) + " as " + str(search_row[0]) + " property")
    
                         


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
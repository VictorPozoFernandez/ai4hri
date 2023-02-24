import rospy
from std_msgs.msg import String
import openai
import os
import numpy as np
from sklearn.cluster import KMeans
from ai4hri.msg import String_list
from keybert import KeyBERT
import mysql.connector
import os
from sklearn.metrics.pairwise import cosine_similarity
os.environ["TOKENIZERS_PARALLELISM"] = "true"

global list_of_lists
list_of_lists=[]


def main():

    rospy.init_node("Ada", anonymous=True)
    rospy.loginfo("Node Ada initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance", String, keyword_callback)
    rospy.Subscriber("/ai4hri/utterance", String, similarity_callback)


    rospy.spin()

def keyword_callback(msg):
        
        kw_model = KeyBERT(model='all-mpnet-base-v2')
        keywords = kw_model.extract_keywords(msg.data, keyphrase_ngram_range=(1,1), use_maxsum=False, top_n=10)

        keyword_list =[]
        for keyword in keywords:
             keyword_list.append(keyword[0])

        if len(list_of_lists) > 1:
             list_of_lists.pop(0)
        list_of_lists.append(keyword_list)

        print("Keywords: " + str(list_of_lists))

        if len(list_of_lists) > 1:
            keyword_list = list_of_lists[0] + list_of_lists[1]
        else:
             keyword_list = list_of_lists[0]

        pub = rospy.Publisher('/ai4hri/keywords', String_list, queue_size= 1) 
        keyword_list_msg = String_list()
        keyword_list_msg = keyword_list
        pub.publish(keyword_list_msg)
        

def similarity_callback(msg):
     
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    model = "text-embedding-ada-002"

    res = openai.Embedding.create( input = [msg.data], engine=model)

    list=[]
    for vec in res["data"]:
        list.append(vec["embedding"])
    
    arr = np.asarray(list)

    utterance_dict = np.load('/home/victor/catkin_ws/src/ai4hri/scripts/utterance_dict.npy',allow_pickle='TRUE').item()
    kmeans = np.load('/home/victor/catkin_ws/src/ai4hri/scripts/kmeans_model.npy',allow_pickle='TRUE').item()
    y = kmeans.predict(arr)

    db = mysql.connector.connect(
    host="localhost",
    user="root",
    password=os.environ.get("MYSQL_PASSWRD"),
    database="Camera_Store"
    )

    mycursor = db.cursor()
    mycursor.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE TABLE_SCHEMA = 'Camera_Store';")

    utterances_to_compare = []
    for row in mycursor:
        utterances_to_compare.append(row[0])

    utterances_to_compare.insert(0,utterance_dict[y[0]])

    res = openai.Embedding.create( input = utterances_to_compare, engine=model)

    embedded_columns=[]
    for vec in res["data"]:
        embedded_columns.append(vec["embedding"])

    utterance = embedded_columns[0]
    embedded_columns.pop(0)
    utterances_to_compare.pop(0)

    scores = []
    for i, column_candidate in enumerate(embedded_columns):
        score = cosine_similarity([utterance],[column_candidate])
        scores.append(score)

    selected_columns = []
    best_scores = sorted(zip(scores, utterances_to_compare), reverse=True)[:3]

    for score in best_scores:
        selected_column= score[1].replace(" ", "_")
        selected_columns.append(selected_column)

    print("Topics: " + str(selected_columns))

    pub = rospy.Publisher('/ai4hri/topics', String_list, queue_size= 1) 
    topic_list_msg = String_list()
    topic_list_msg = selected_columns
    pub.publish(topic_list_msg)

if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
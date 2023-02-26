import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import os
import numpy as np
import mysql.connector
from collections import Counter
import random
from keybert import KeyBERT

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
    
    keywords = msg.data

    for keyword in msg.data:

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

    print("Camera of reference: " + camera_reference[1]) #See if there is a problem with paralel·l execution of ros nodes. If there is, execute nodes in a streamline way (see notes photo)
    
    mycursor2.execute("SELECT column_name FROM information_schema.columns WHERE table_schema = 'Camera_Store';")
    
    for search_row in mycursor2:

        if search_row[0] in msg.data:
            mycursor3.execute("SELECT table_name FROM information_schema.columns WHERE column_name = %s",search_row)
            
            for row2 in mycursor3:

                print("Searching column '" + search_row[0] + "' in table '" + str(row2[0])+"':")
                mycursor4.execute("SELECT " + search_row[0] + " FROM " + row2[0] + " WHERE Product_ID = %s", (camera_reference[0],))

                for finding in mycursor4:

                    if isinstance(finding[0], float) or isinstance(finding[0], int):
                        
                        if str(int(finding[0])) in keywords:
                            print(str(finding[0]) + " found!")
                    
                    else:

                        count = 0
                        digit_condition = False
                        finding_keywords = kw_model.extract_keywords(finding[0], keyphrase_ngram_range=(1,1), use_maxsum=False, top_n=10)

                        for keyword in keywords:

                            if keyword.isdigit():
                                digit_condition = True

                                if keyword.lower() in finding[0].lower():
                                    digit_condition = False
                                    count +=1

                            elif keyword.lower() in finding[0].lower():
                                count +=1

                        if (digit_condition == False) and ((count/len(finding_keywords))>0.60):
                            print(finding[0] + " found!")
                    


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
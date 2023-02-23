import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import os
import numpy as np
import mysql.connector
from collections import Counter
import random

db = mysql.connector.connect(
  host="localhost",
  user="root",
  password=os.environ.get("MYSQL_PASSWRD"),
  database="Camera_Store"
)
mycursor = db.cursor()

def main():

    rospy.init_node("MySQL", anonymous=True)
    rospy.loginfo("Node MySQL initialized. Listening...")
    rospy.Subscriber("/ai4hri/real_keywords", String_list, callback)

    rospy.spin()

def callback(msg):

    mycursor.execute("SELECT Model FROM Camera")

    models = []
    for row in mycursor:
        models.append(row[0])

    possible_models = []
    possible_models_2 = []    
    
    for keyword in msg.data:

        for row in models:

            if keyword in row.lower():
                possible_models.append(row)

    if len(possible_models)!=0:

        counts = Counter(possible_models)
        max_count = counts.most_common(1)[0][1]

        for value, count in counts.most_common():
            if count == max_count:
                possible_models_2.append(value)
        
        if len(possible_models_2) == 1:
            model = possible_models_2[0] 
        
        else:
            #Look the costumer and shopkeeper position to identify the camera they are refering to. For now, we choose random from the possible_models_2 list
            model = random.choice(possible_models_2)


    else:
        #Look the costumer and shopkeeper position to identify the camera they are refering to. For now, we choose random from the models list
        model = random.choice(models)

    # if model =! camera closest to costumer and shopkeeper position:
            #model2 = (Look the costumer and shopkeeper position to identify the camera they are using as comparison)

    print(model)
    #print(model2)



         

if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
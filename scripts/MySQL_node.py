import rospy
from ai4hri.msg import String_list
import os
import mysql.connector
from keybert import KeyBERT

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

    num_models = int(msg.data[-2])/2
    num_topics = msg.data[-1]
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

    keywords_interest = msg.data
    
    mycursor2.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE table_schema = 'Camera_Store' AND TABLE_NAME IN (SELECT table_name FROM information_schema.columns WHERE column_name = 'Product_ID')")
 
    for search_row in mycursor2:

        if search_row[0] in topics_interest:
            mycursor3.execute("SELECT table_name FROM information_schema.columns WHERE column_name = %s",search_row)
            
            for row2 in mycursor3:
                
                if row2[0] == 'replication_asynchronous_connection_failover':
                    pass
                
                else:
                    if DEBUG == True: 
                        print("")
                        print("Searching column '" + search_row[0] + "' in table '" + str(row2[0])+"':")

                    for model_reference in models_interest:

                        mycursor4.execute("SELECT " + search_row[0] + " FROM " + row2[0] + " WHERE Product_ID = %s", (model_reference[0],))

                        for finding in mycursor4:

                            finding_keywords = kw_model.extract_keywords(str(finding[0]), keyphrase_ngram_range=(1,1), use_maxsum=False, top_n=10)
                            count = 0

                            for keyword in keywords_interest:

                                if keyword.lower() in str(finding[0]).lower(): 
                                    count +=1
                            
                            if ((count/len(finding_keywords))>0.60):
                                print("KNOWLEDGE DETECTION: The shopkeeper knows that the " + str(model_reference[1]) + " model has " + str(finding[0]) + " as " + str(search_row[0]) + " property")

                         
if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
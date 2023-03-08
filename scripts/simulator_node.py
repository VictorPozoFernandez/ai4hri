import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import os
import pandas as pd
import numpy as np

#DEBUG = rospy.get_param('/whisper/DEBUG')

def main():

    rospy.init_node("simulator", anonymous=True)
    rospy.loginfo("Node simulator initialized...")
    pub = rospy.Publisher('/ai4hri/utterance_and_position', String_list, queue_size= 10) 
    rate = rospy.Rate(1)


    while not rospy.is_shutdown():

        num_interaction = input("Select interaction: ")

        df = pd.read_csv("/home/victor/catkin_ws/src/ai4hri/simulated data.csv")
        extracted_columns = df.iloc[:,[5, 14, 16, 25, 35]]
        extracted_columns = extracted_columns.dropna()
        interaction = extracted_columns[extracted_columns['TRIAL'] == int(num_interaction)]
        interaction_no_trial = interaction.iloc[:,1:]
        
        num_rows = len(interaction_no_trial)

        for row in range(num_rows):

            utterance = String_list()
            utterance = interaction_no_trial.iloc[row,:].to_list()
            pub.publish(utterance) 
            print(interaction_no_trial.iloc[row,:].to_list())
            
            for _ in range(6):
                rate.sleep()




if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
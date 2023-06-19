import rospy
from std_msgs.msg import String
import pandas as pd
import os

def main():

    # Initialize the simulator ROS node and a publisher for the AI4HRI utterance_and_position topic
    rospy.init_node("simulator", anonymous=True)
    rospy.loginfo("Node simulator initialized...")
    pub = rospy.Publisher('/ai4hri/utterance', String, queue_size= 10) 
    rate = rospy.Rate(1)

    # Main loop that runs until the ROS node is shut down.
    while not rospy.is_shutdown():
        
        for _ in range(5):
            rate.sleep()

        # Clear the console.
        for _ in range(30):
            print("")

        # Prompt user to select an interaction.
        num_interaction = input("Select interaction: ")

        # Read the simulated data from a CSV file and extract the relevant columns.
        script_dir = os.path.dirname(os.path.abspath(__file__))  # Returns the absolute path to the script
        parent_dir = os.path.dirname(script_dir)
        file_path = os.path.join(parent_dir, 'simplified_database.csv')
        print(file_path)
        df = pd.read_csv(file_path)

        # Filter the extracted columns by the selected interaction number.
        
        interaction = df[df['TRIAL'] == str(num_interaction)]
        interaction_no_trial = interaction.iloc[:,1]
        
        # Iterate through the rows of the selected interaction. Publish the message to the "/ai4hri/utterance" topic.
        num_rows = len(interaction_no_trial)
                         
        for row in range(num_rows):

            utterance = String()
            utterance = interaction_no_trial.iloc[row]
            pub.publish(utterance) 
            print("------------------------------------------")
            print(interaction_no_trial.iloc[row])
            
            x = input()
            if x == "q":
                break


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
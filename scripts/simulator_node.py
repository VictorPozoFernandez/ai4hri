import rospy
from ai4hri.msg import String_list
import pandas as pd

DEBUG = rospy.get_param('/whisper/DEBUG')
SIMULATOR = rospy.get_param('/whisper/SIMULATOR')


def main():

    # Initialize the simulator ROS node and a publisher for the AI4HRI utterance_and_position topic
    rospy.init_node("simulator", anonymous=True)
    rospy.loginfo("Node simulator initialized...")
    pub = rospy.Publisher('/ai4hri/utterance_and_position', String_list, queue_size= 10) 
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
        df = pd.read_csv("/home/victor/catkin_ws/src/ai4hri/simulated data.csv")
        extracted_columns = df.iloc[:,[5, 14, 16, 25, 35]]
        extracted_columns = extracted_columns.dropna()

        # Filter the extracted columns by the selected interaction number.
        interaction = extracted_columns[extracted_columns['TRIAL'] == int(num_interaction)]
        interaction_no_trial = interaction.iloc[:,1:]
        
        # Iterate through the rows of the selected interaction. Publish the message to the "/ai4hri/utterance_and_position" topic.
        num_rows = len(interaction_no_trial)
        for row in range(num_rows):

            utterance = String_list()
            utterance = interaction_no_trial.iloc[row,:].to_list()
            pub.publish(utterance) 
            print("------------------------------------------")
            print(interaction_no_trial.iloc[row,:].to_list())
            
            for _ in range(10):
                rate.sleep()


if __name__ == '__main__':

    try:
        if SIMULATOR == True:
            main()
    
    except rospy.ROSInterruptException:
        pass
    
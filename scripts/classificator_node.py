import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import openai
import os

DEBUG = rospy.get_param('/classificator/DEBUG')


def main():

    # Initialize the classficator ROS node and subscribe to utterance topic
    rospy.init_node("classificator", anonymous=True)
    rospy.loginfo("Node classificator initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance", String, callback)
    
    rospy.spin()


def callback(msg):

    # Initialize publisher for the utterance_and_position topic
    pub = rospy.Publisher('/ai4hri/utterance_and_position', String_list, queue_size= 1) 
    global new_utterance
    utterance_and_position = String_list()

    # Store the previous utterance if available
    try:
        previous_utterance = new_utterance
    except:
        previous_utterance = ""
    
    # Get new utterance from message and classify it
    new_utterance = msg.data
    classification_result = sentence_classification(new_utterance)
    print("------------------------------------------")
    print(" - " + new_utterance + " (" + classification_result + ")")

    if "Shopkeeper" in classification_result:
        # Get the position of the shopkeeper and the customer using a position tracker
        utterance_and_position = get_current_position(previous_utterance, new_utterance)
        
        if DEBUG == True:
            print("")
            print("Publishing " + str(utterance_and_position))

        # Publish the utterance and position
        pub.publish(utterance_and_position)


def sentence_classification(new_utterance):

    # Set OpenAI API credentials
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    # Request classification from fine-tuned GPT3 model
    classification_result = openai.Completion.create(
        model="ada:ft-personal-2023-03-07-22-12-15", #Change to your own fine-tuned model's name
        prompt= new_utterance + " ### ", #Change if needed (depending on which stop message for the prompt did you use while training the fine-tuned model)
        stop="END") #Change if needed (depending on which stop message did you use for the generated output while training the fine-tuned model)
    
    return classification_result["choices"][0]["text"]


def get_current_position(previous_utterance, utterance):

    #Pending to implement a position tracker. Default to NULL
    localization1 = "NULL"
    localization2 = "NULL"

    # Create a list to store the previous and current utterances along with the current positions of the customer and shopkeeper
    utterance_and_position = []
    utterance_and_position.append(previous_utterance)
    utterance_and_position.append(utterance)
    utterance_and_position.append(localization1)
    utterance_and_position.append(localization2)

    return utterance_and_position


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    

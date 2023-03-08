import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import speech_recognition as sr
import whisper
import queue
import threading
import torch
import numpy as np
import openai
import os

#DEBUG = rospy.get_param('/whisper/DEBUG')

pub = rospy.Publisher('/ai4hri/utterance_and_position', String_list, queue_size= 1) 

def main():

    rospy.init_node("classificator", anonymous=True)
    rospy.loginfo("Node classificator initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance", String, callback)
    
    rospy.spin()
    
def callback(msg):

    global new_utterance
    utterance_and_position = String_list()

    try:
        previous_utterance = new_utterance
    except:
        previous_utterance = ""
    
    new_utterance = msg.data
    classification_result = sentence_classification(new_utterance)
    print("--------------------------------")
    print("The utterance '" + new_utterance + "' comes from the" + classification_result)

    if "Shopkeeper" in classification_result:

        utterance_and_position = get_current_position(previous_utterance, new_utterance)
        print("Publishing " + str(utterance_and_position))

        pub.publish(utterance_and_position)


def sentence_classification(new_utterance):

    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    classification_result = openai.Completion.create(
        model="ada:ft-personal-2023-03-07-22-12-15",
        prompt= new_utterance + " ### ",
        stop="END")
    
    return classification_result["choices"][0]["text"]


def get_current_position(previous_utterance, utterance):

    localization1 = "NULL"
    localization2 = "NULL"

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
    

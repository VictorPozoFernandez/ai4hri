import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import openai
import os

#DEBUG = rospy.get_param('/whisper/DEBUG')

system_message = """Imagine you are helping me to identify the camera model that is being discussed between a camera shopkeeper and a customer. You are required to output the following answers:

- Camera model: All possible camera models that fit best in the context of the current and previous conversations. 

You have the following camera models to choose from. You are not to use any other hypothetical camera models:

- Nikon Coolpix S2800
- Sony Alpha a6000
- Canon EOS 5D Mark III

Here is an example conversation that illustrates how can you output your answer. The interactions appear in cronological order and together they represent a full conversation between the camera shopkeeper and the costumer:

CUSTOMER: <Customer sentence> SHOPKEEPER: <Shopkeeper sentence>
CUSTOMER: <Customer sentence> SHOPKEEPER: <Shopkeeper sentence>
You: CAMERA MODEL - <all possible models of the cameras> 

Output only the possible models of the cameras that are being discussed in the last interaction. Remember to use the previous interactions to have more context about the camera model that is being discussed right now.
In case of uncertainty about the camera model that is being discussed, output all the possible camera models instead.



"""

messages_history=[
    {"role": "system", "content": system_message}
    ]


def main():

    rospy.init_node("GPT", anonymous=True)
    rospy.loginfo("Node GPT initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance", String, callback)
    rospy.Subscriber("/ai4hri/utterance_simulator", String_list, callback_simulator)

    rospy.spin()

def callback(msg):
        
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")
    messages_history.append({"role": "user", "content": msg.data})


    completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo", 
    messages=messages_history,
    temperature=0.0
    )

    print("--------------------------------------------")
    print(msg.data)
    print(completion["choices"][0]["message"]["content"])

    pub = rospy.Publisher('/ai4hri/detected_models', String, queue_size= 1) 
    detected_models= String()
    detected_models = completion["choices"][0]["message"]["content"]
    pub.publish(detected_models)

def callback_simulator(msg):
        
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    print(msg.data)


    '''messages_history.append({"role": "user", "content": msg.data})


    completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo", 
    messages=messages_history,
    temperature=0.0
    )

    print("--------------------------------------------")
    print(msg.data)
    print(completion["choices"][0]["message"]["content"])

    pub = rospy.Publisher('/ai4hri/detected_models', String, queue_size= 1) 
    detected_models= String()
    detected_models = completion["choices"][0]["message"]["content"]
    pub.publish(detected_models)'''



if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
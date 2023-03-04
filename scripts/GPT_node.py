import rospy
from std_msgs.msg import String
import openai
import os

#DEBUG = rospy.get_param('/whisper/DEBUG')

system_message = """Imagine you are helping me to identify the camera model that is being discussed between a camera shopkeeper and a customer. You are required to output the following answers:

- Camera model: All possible camera models that fit bests with the context of the conversation and previous conversations. 
- Reason: After you output the Camera model answer, you should explain why you did what you did. 

You have the following camera models to choose from. You are not to use any other hypothetical camera models:

- Nikon Coolpix S2800
- Sony Alpha a6000
- Sony Alpha a5000
- Sony DSC-RX100
- Canon EOS 5D Mark III

Here is an example scenario that illustrates how can you output your answer.:

Customer: <Customer sentence> Shopkeeper: <Shopkeeper sentence>
You: Camera Model - <chosen camera model>
You: Reason - <explanation>

Remember to use the previous conversations to have more context of the camera model that is being discussed.

"""

messages_history=[
    {"role": "system", "content": system_message}
    ]


def main():

    rospy.init_node("GPT", anonymous=True)
    rospy.loginfo("Node GPT initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance", String, callback)

    rospy.spin()

def callback(msg):
        
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")
    messages_history.append({"role": "user", "content": msg.data})


    completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo", 
    messages=messages_history,
    temperature=0
    )

    print(completion["choices"][0]["message"]["content"])
    messages_history.append({"role": "assistant", "content": completion["choices"][0]["message"]["content"]})
    


        
    

if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
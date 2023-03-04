import rospy
from std_msgs.msg import String
import openai
import os

#DEBUG = rospy.get_param('/whisper/DEBUG')

system_message = """Imagine you are helping me to identify the camera of conversation between a camera shopkeeper and a customer. You are required to output the following answers:

- Camera model: The model of the camera that fits bests with the context of the conversation.
- Reason: After you output the Camera model answer, you should explain why you did what you did. 

You have the following camera models to choose from. You are not to use any other hypothetical camera models:

- Nikon Coolpix S2800
- Sony Alpha a6000
- Sony Alpha a5000
- Sony DSC-RX100
- Canon EOS 5D Mark III

Here is an example scenario that illustrates how can you output your answers:

Customer: Hello, I would like to buy a camera. Shopkeeper: This is the nikon camera.
You: Camera Model - Nikon Coolpix S2800
You: Reason - The Shopkeeper presented the Nikon camera model.

Customer: And what about this one? Shopkeeper: This is the sony alpha camera. Its also very professional.
You: Camera Model - Sony Alpha a6000 - Sony Alpha a5000
You: Reason - The Shopkeeper presented the sony alpha camera model. The camera model of conversation can be either the a6000 model or the a5000 model. 

Customer: Is this camera better than the previous one? Shopkeeper: No, the other was better, but it's more expensive
You: Camera Model - Sony Alpha a6000 - Sony Alpha a5000 - Nikon Coolpix S2800 
You: Reason - The customer compared the last camera of conversation (either the Sony Alpha a6000 or the Sony Alpha a5000) and the one that was shown at the beggining of the conversation (Nikon Coolpix S2800)

Customer: Which model is? Shopkeeper: This is the a6000 model.
You: Camera Model - Sony Alpha a6000
You: Reason - The Shopkeeper identified that the camera model of conversation was the a6000 model.

Costumer: I see. Do you have anything else? Shopkeeper: I also have the Canon camera, which is also very affordable
You: Camera Model - Canon EOS 5D Mark III
You: Reason - The Shopkeeper presented the Canon camera model.

Costumer: Which camera do you recommend me then? Shopkeeper: I recommend you to choose among the two most affordable of the three that we have seen.
You: Camera Model - Canon EOS 5D Mark III - Sony Alpha a6000
You: Reason - The Shopkeeper presented the Nikon Coolpix S2800, the Sony Alpha a6000 and the Canon EOS 5D Mark III. Among these, he said that the Nikon was more expensive, and the others were more affordable. Therefore, the cameras of conversation are the Canon and the Sont Alpga a6000.

Remember to consider previous conversations of the costumer and the shopkeeper to extract the Camera model from the context."""

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
    messages=messages_history
    )

    print(completion["choices"][0]["message"]["content"])
    messages_history.append({"role": "assistant", "content": completion["choices"][0]["message"]["content"]})
    


        
    

if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import openai
import os
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage
from langchain.prompts import PromptTemplate
import re
import json

GPT4 = rospy.get_param('/classificator/GPT4')

global toggle
global previous_utterances
toggle = False
previous_utterances = ""


def main():

    # Initialize the classficator ROS node and subscribe to utterance topic
    rospy.init_node("classificator", anonymous=True)
    rospy.loginfo("Node classificator initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance", String, callback)
    
    rospy.spin()


def callback(msg):

    global new_utterance
    global toggle

    # Initialize publisher for the utterance_and_position topic
    pub = rospy.Publisher('/ai4hri/utterance_and_position', String_list, queue_size= 1) 
    utterance_and_position = String_list()

    # Store the previous utterance if available
    try:
        previous_utterance = new_utterance
    except:
        previous_utterance = ""
    
    # Get new utterance from message and classify it
    new_utterance = msg.data

    # In GPT4 mode we manually use GPT4 instead of the Fine-Tuned GPT3
    if GPT4 == True:
        classification_result = sentence_classification_chatgpt(new_utterance)     
    else:
        classification_result = sentence_classification(new_utterance)

    print("------------------------------------------")
    print("\033[94m - " + new_utterance + " (" + classification_result + ") \033[0m")
    print("")

    #The toggle variable is used to detect if the Shopkeeper speaks twice in a row. In that case, the previous utterance is sent empty indicating that the Customer hasn't spoken in between.
    
    if (("Shopkeeper" in classification_result) or ("shopkeeper" in classification_result)) and (toggle == False):
        # Get the position of the shopkeeper and the customer using a position tracker
        utterance_and_position = get_current_position(previous_utterance, new_utterance)

        # Publish the utterance and position
        pub.publish(utterance_and_position)
        toggle = True
    
    elif (("Shopkeeper" in classification_result) or ("shopkeeper" in classification_result)) and (toggle == True):
        # Get the position of the shopkeeper and the customer using a position tracker
        previous_utterance = ""
        utterance_and_position = get_current_position(previous_utterance, new_utterance)

        # Publish the utterance and position
        pub.publish(utterance_and_position)
        toggle = True
    
    else:
        toggle = False


def sentence_classification(new_utterance):

    # Set OpenAI API credentials
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")

    # Request classification from fine-tuned GPT3 model
    classification_result = openai.Completion.create(
        model="ada:ft-personal-2023-03-29-12-35-58", #Improved model with syntetic data
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


def sentence_classification_chatgpt(new_utterance):

    global previous_utterances

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")

    # Prepare prompt to send, using JSON format
    chat = ChatOpenAI(model_name="gpt-4", temperature=0, openai_api_key=openai_api_key)


    system_prompt = """
    You are a helpful assistant in a Camera Shop that identifies if the 'New Interaction' of a conversation history is said by a Customer or the Shopkeeper.

    Customer's behaviour:

    The customer enters the shop, typically looking around at the various displays of cameras, lenses, and other photography equipment.
    If they need more information or can't find a specific item, they will approach the shopkeeper or sales associate.
    Once they get help, they might want to test some cameras or lenses based on the shop's policy.
    If they find what they're looking for and are satisfied with it, they'll proceed to purchase it. Otherwise, they may thank the shopkeeper for their help and leave.
    
    Shopkeeper's behaviour:

    When a customer enters, the shopkeeper typically greets them, offering initial assistance.
    They stay available for any questions or assistance, ready to explain the features, specifications, and pricing of the products.
    If the customer wants to test a camera, the shopkeeper assists them, explaining how to use it correctly.
    If the Shopkeeper is not able to answer to a question given by the customer, he may call for help.
    Once the customer decides to purchase, the shopkeeper helps them with the transaction, processes their payment, and packages their purchase.
    They may also explain the return policy, warranty, or any other after-sales services, thank the customer, and invite them back to the store.

    Here is an example that illustrates how can you output your answer.

    ## New Interaction: 'In which colors is available this model?';
    You: {"Detection": "Customer"}

    Shopkeeper: 'This camera costs 100 dollars' ## Shopkeeper: 'The model is available in color white and blue' ## New Interaction: 'How much does it weight?';
    You: {"Detection": "Customer"}
    
    Shopkeeper: 'This camera costs 100 dollars' ## Customer: 'And the price of the first camera that you showed me?' ## New Interaction: '68 dollars only, it's very affordable';
    You: {"Detection": "Shopkeeper"}

    Output only with the labels "Customer" or "Shopkeeper"
    Output only the result of the New Interaction.
    Output the answer only in JSON format.
    """

    user_template = """
    {previous_utterances} ## New Interaction: {new_utterance}
    """

    user_prompt_template = PromptTemplate(input_variables=["previous_utterances", "new_utterance"], template=user_template)
    user_prompt = user_prompt_template.format(previous_utterances = previous_utterances, new_utterance = new_utterance)

    prompt_history = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_prompt)
    ]

    result = chat(prompt_history)
    data = extract_json(result.content)

    previous_utterances = previous_utterances + " ## " + data["Detection"] + ": " + new_utterance
    
    return data["Detection"]


def extract_json(s):
    json_match = re.search(r'\{.*\}', s, re.DOTALL)
    if json_match:
        try:
            data = json.loads(json_match.group())
            return data
        except json.JSONDecodeError:
            print("Invalid JSON")
            return None
    else:
        print("No JSON found in the string")
        return {"Detection": "null", "Model": "null", "Output" : "null",}


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    

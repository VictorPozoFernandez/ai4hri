import rospy
from std_msgs.msg import String
from ai4hri.msg import String_list
import openai
import os
import mysql.connector

db = mysql.connector.connect(
  host="localhost",
  user="root",
  password=os.environ.get("MYSQL_PASSWRD"),
  database="Camera_Store"
)
mycursorGPT = db.cursor(buffered=True)
mycursorGPT2 = db.cursor(buffered=True)
mycursorGPT3 = db.cursor(buffered=True)

close_product_IDs = [1,2,3]
characteristics_product_IDs = []

for Product_ID in close_product_IDs:

    characterstics_model = []
    mycursorGPT.execute("SELECT table_name FROM information_schema.columns WHERE column_name = 'Product_ID'")

    for table_name in mycursorGPT:

        mycursorGPT2.execute("SELECT COLUMN_NAME FROM INFORMATION_SCHEMA.COLUMNS WHERE TABLE_NAME = '" + table_name[0] + "'")
        
        for column_name in mycursorGPT2:
        
            if column_name[0] != 'Product_ID':

                mycursorGPT3.execute("SELECT " + column_name[0]+ " FROM " + table_name[0] + " WHERE Product_ID = " + str(Product_ID))

                column = []
                for characteristic in mycursorGPT3:

                    column.append(characteristic[0])
                
                characterstics_model.append((column_name[0], column))

            
    characteristics_product_IDs.append(characterstics_model)


system_message = """Imagine you are helping me to identify the camera model that a shopkeeper is presenting to a customer. You are required to output the following answers:

- Camera model: All possible camera models that the shopkeeper is presenting.

You have the following camera models to choose from. You are not to use any other hypothetical camera models:

- Nikon Coolpix S2800. Characteristics (Point and Shoot/ color purple pink black silver red/ weight 120 grams/ 18 preset modes/ 12 glamour retouch effects/ price 68/ resolution 20.1 megapixels/ optical zoom 5x)
- Sony Alpha a6000. Characteristics (Mirrorless/ color white silver black/ weight 470 grams/ 9 preset modes/ 13 artistic effect modes/ price 550/ resolution 24 megapixels/ ISO 3200)
- Sony Alpha a5000. Characteristics (Mirrorless/ color black/ weight 400 grams/ 5 preset modes/ 13 artistic effect modes/ price 300/ resolution 24 megapixels/ ISO 2000)
- Canon EOS 5D Mark III. Characteristics (DSLR/ color black/ weight 950/ price 2000/ resolution 22.3 megapixels/ ISO 6400/ silent shooting)

Here is an example conversation that illustrates how can you output your answer. The interactions appear in cronological order, and together they represent a full conversation between the camera shopkeeper and the costumer:

CUSTOMER: <Customer sentence> SHOPKEEPER: <Shopkeeper sentence>
CUSTOMER: <Customer sentence> SHOPKEEPER: <Shopkeeper sentence>
You: CAMERA MODEL - <all possible camera models> 

Remember to use the previous interactions to have more context about all the possible camera models that the shopkeeper is presenting right now.
Output only the camera model that the shopkeeper is presenting in the last interaction. 
Aproximate the characteristics of the camera that appear in <Shopkeeper sentence> with the characteristics of the camera models to identify them.

"""

message1 = """Imagine you are helping me to identify the camera model that a shopkeeper is presenting to a customer. You are required to output the following answers:

- Camera model: All possible camera models that the shopkeeper is presenting.

You have the following camera models to choose from. You are not to use any other hypothetical camera models:

"""

message2 = """ 

Here is an example conversation that illustrates how can you output your answer. The interactions appear in cronological order, and together they represent a full conversation between the camera shopkeeper and the costumer:

CUSTOMER: <Customer sentence> SHOPKEEPER: <Shopkeeper sentence>
CUSTOMER: <Customer sentence> SHOPKEEPER: <Shopkeeper sentence>
You: CAMERA MODEL - <all possible camera models> 

Remember to use the previous interactions to have more context about all the possible camera models that the shopkeeper is presenting right now.
Output only the camera model that the shopkeeper is presenting in the last interaction. 
Aproximate the characteristics of the camera that appear in <Shopkeeper sentence> with the characteristics of the camera models to identify them.
"""

system_message2 = message1 + str(characteristics_product_IDs) + message2

print(system_message2)

messages_history=[
    {"role": "system", "content": system_message2}
    ]


def main():

    rospy.init_node("GPT", anonymous=True)
    rospy.loginfo("Node GPT initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance_simulator", String_list, callback)

    rospy.spin()

def callback(msg):
        
    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")
    messages_history.append({"role": "user", "content": "CUSTOMER: " + msg.data[0] + " SHOPKEEPER: " + msg.data[1]})

    if len(messages_history) > 4:
        messages_history.pop(1)


    completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo", 
    messages=messages_history,
    temperature=0.0
    )

    print("--------------------------------------------")
    print("CUSTOMER: " + msg.data[0] + " SHOPKEEPER: " + msg.data[1])
    
    print(completion["choices"][0]["message"]["content"])


    pub = rospy.Publisher('/ai4hri/detected_models', String, queue_size= 1) 
    detected_models= String()
    detected_models = completion["choices"][0]["message"]["content"]
    pub.publish(detected_models)


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
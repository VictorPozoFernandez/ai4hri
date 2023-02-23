import rospy
from std_msgs.msg import String
import os
import numpy as np


def main():

    rospy.init_node("MySQL", anonymous=True)
    rospy.loginfo("Node MySQL initialized. Listening...")
    rospy.Subscriber("/ai4hri/keywords", String, callback)

    rospy.spin()

def callback(msg):

    print(msg.data)
    keywords = list(msg.data.split(","))
    for keyword in keywords:
        print(keyword)
         

if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
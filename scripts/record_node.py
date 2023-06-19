import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import speech_recognition as sr

global audio_control
audio_control = "RESUME"

def callback(msg):
    global audio_control
    audio_control = msg.data

def record_audio(pub, energy, pause):

    global audio_control

    # Initialize the recognizer and set energy and pause thresholds
    r = sr.Recognizer()
    r.energy_threshold = energy
    r.pause_threshold = pause

    # Open the microphone with the specified sample rate and device index
    counter = 0
    found_micro = False
    while (counter < 15) and (found_micro == False) :
        try:      
            with sr.Microphone(sample_rate=16000,  device_index=counter) as source:

                # Clear the console
                for x in range(4):
                    print("")
                rospy.loginfo("Node record_node initialized. Listening...")

                # Record audio while the ROS node is running
                while not rospy.is_shutdown():
                    audio = r.listen(source)
                    wav_data = audio.get_wav_data()

                    if (audio_control == "RESUME"):
                        pub.publish(wav_data)

                    found_micro = True
        except:
            counter += 1
            if counter >= 15:
                print("Microphone not found")
                break

def main():
    # Initialize the whisper ROS node and a publisher for the AI4HRI utterance topic
    rospy.init_node("audio_recorder", anonymous=True)
    pub = rospy.Publisher('/ai4hri/audio_data', AudioData, queue_size= 1)
    rospy.Subscriber("/ai4hri/audio_control", String, callback)

    energy = 25000
    pause = 0.5

    # Print available microhpones PC
    for index, name in enumerate(sr.Microphone.list_microphone_names()): print(f'{index}, {name}')

    record_audio(pub, energy, pause)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass


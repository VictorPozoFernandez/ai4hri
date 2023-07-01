import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import speech_recognition as sr
from dynamic_reconfigure.server import Server
from ai4hri.cfg import audiorecorder_ai4hriConfig

global audio_control
audio_control = "STOP"

def callback(msg):
    global audio_control
    audio_control = msg.data

def callback2(config, level):

    rospy.loginfo("""Reconfigure Request: {int_energy} """.format(**config))
    
    global energy
    energy=config.int_energy

    return config

def record_audio(pub):

    global audio_control
    global energy

    # Initialize the recognizer and set energy and pause thresholds
    r = sr.Recognizer()
    r.energy_threshold = energy
    r.pause_threshold = 0.5

    try:      

         # Print available microphones PC

        rospy.sleep(2)
        print("")
        for index, name in enumerate(sr.Microphone.list_microphone_names()): print(f'{index}, {name}')
        print('')
        counter = input('Choose a microphone device: ')

        with sr.Microphone(sample_rate=16000,  device_index=int(counter)) as source:

            print("")
            mode = input('Choose a recording mode (1.Automatic, 2.Manual): ')
            
            # Clear the console
            for x in range(4):
                print("")
            rospy.loginfo("Node record_node initialized. Listening...")

            if mode == "1":
                # Record audio while the ROS node is running
                while not rospy.is_shutdown():
                    audio = r.listen(source)
                    wav_data = audio.get_wav_data()
                    pub.publish(wav_data)
            
            if mode == "2":
                complete_audio=None

                # Record audio while the ROS node is running
                while not rospy.is_shutdown():
                    
                    audio = r.record(source, duration=1)  # Record for a small duration

                    if audio_control == "RESUME":

                            # Concatenate audio chunks if any speech is detected
                        if complete_audio == None:
                            complete_audio = sr.AudioData(audio.frame_data, audio.sample_rate, audio.sample_width)
                        else:
                            complete_audio = sr.AudioData(complete_audio.frame_data + audio.frame_data, audio.sample_rate, audio.sample_width)
                    
                    elif (audio_control == "STOP") and (complete_audio != None):              
                        
                        complete_audio = sr.AudioData(complete_audio.frame_data + audio.frame_data, audio.sample_rate, audio.sample_width)
                        wav_data = complete_audio.get_wav_data()
                        pub.publish(wav_data)
                        complete_audio = None
                        
                    else:
                        previous_audio = audio
                        complete_audio = None

    
    except Exception as e:
        print(e)
        print("Microphone not found")
                

def main():
    # Initialize the whisper ROS node and a publisher for the AI4HRI utterance topic
    rospy.init_node("audio_recorder", anonymous=True)
    pub = rospy.Publisher('/ai4hri/audio_data', AudioData, queue_size= 1)
    rospy.Subscriber("/ai4hri/audio_control", String, callback)
    srv = Server(audiorecorder_ai4hriConfig, callback2)

    record_audio(pub)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass


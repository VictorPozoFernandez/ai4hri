import rospy
from std_msgs.msg import String
import speech_recognition as sr
import whisper
import queue
import threading
import torch
import numpy as np



def main():

    rospy.init_node("whisper", anonymous=True)

    pub = rospy.Publisher('/ai4hri/utterance', String, queue_size= 1) 

    model = "small.en"  # ["tiny.en","base.en", "small.en","medium.en","large"]   Use the "large" model for detecting different languages other than English. 
    energy = 300
    pause = 0.5

    ## Microphone detection. Uncomment to list available microphones
    #for index, name in enumerate(sr.Microphone.list_microphone_names()): print(f'{index}, {name}')

    audio_model = whisper.load_model(model)
    audio_queue = queue.Queue()
    result_queue = queue.Queue()
    rate = rospy.Rate(1)

    threading.Thread(target=record_audio,
                    args=(audio_queue, energy, pause, rate)).start()

    threading.Thread(target=transcribe_audio,
                    args=(audio_queue, result_queue, audio_model, rate)).start()

    while not rospy.is_shutdown():

        utterance = String()
        utterance = result_queue.get() 
        if len(utterance) > 15:
            print("---------------------") 
            print(utterance)
            modified_utterance = utterance.replace(",", "")
            pub.publish(modified_utterance)
        rate.sleep()
    
    


def record_audio(audio_queue, energy, pause, rate):

    r = sr.Recognizer()
    r.energy_threshold = energy
    r.pause_threshold = pause

    with sr.Microphone(sample_rate=16000,  device_index=13) as source:
        rospy.loginfo("Node whisper initialized. Listening...")
        
        while not rospy.is_shutdown():
            audio = r.listen(source)
            audio_data = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
            audio_queue.put_nowait(audio_data)
            rate.sleep()


def transcribe_audio(audio_queue, result_queue, audio_model, rate):
    
    while not rospy.is_shutdown():
        audio_data = audio_queue.get()
        result = audio_model.transcribe(audio_data,language='english')
        predicted_text = result["text"]
        result_queue.put_nowait(predicted_text)
        rate.sleep()


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    

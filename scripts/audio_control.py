import rospy
from pynput import keyboard
from std_msgs.msg import String

audio_control = "STOP"
keyword = 'p'
publish_once = False

def on_press(key):
    global audio_control, publish_once
    try:
        if key.char == keyword:
            if audio_control == "RESUME":
                audio_control = "STOP"
            else:
                audio_control = "RESUME"
            publish_once = True
    except AttributeError:
        pass  # Ignore special keys like ctrl, alt, etc.

def audio_control_publisher():
    rospy.init_node('audio_control_publisher', anonymous=True)
    pub = rospy.Publisher('/ai4hri/audio_control', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    global publish_once

    while not rospy.is_shutdown():
        if publish_once:
            msg = String()
            msg.data = audio_control
            pub.publish(msg)
            publish_once = False
        rate.sleep()

if __name__ == "__main__":
    try:
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        audio_control_publisher()
    except rospy.ROSInterruptException:
        pass

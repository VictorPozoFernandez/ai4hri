FROM osrf/ros:noetic-desktop-full-focal

RUN apt update && \
    apt upgrade -y && \
    apt install git -y && \
    apt install python3-catkin-tools -y && \
    apt install python3-pip -y && \
    apt install ros-noetic-audio-common -y && \
    apt install portaudio19-dev python-all-dev alsa-base alsa-utils -y && \
    mkdir -p /catkin_ws/src && \
    git clone https://github.com/VictorPozoFernandez/ai4hri.git /catkin_ws/src/ai4hri && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make; . /catkin_ws/devel/setup.bash' && \
    echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc && \
    echo 'export ROS_MASTER_URI=http://192.168.203.36:11311/' && \
    echo 'echo ROS_IP=172.17.0.2'

WORKDIR /catkin_ws/src/ai4hri

RUN pip install -r requirements.txt

CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
                         source /catkin_ws/devel/setup.bash && \
                         roslaunch ai4hri knowledge_recognition.launch"]
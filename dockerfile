FROM osrf/ros:noetic-desktop-full-focal

RUN apt update
RUN apt upgrade -y
RUN apt install git -y
RUN source /opt/ros/noetic/setup.bash
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
RUN mkdir catkin_ws
RUN cd catkin_ws
RUN mkdir src
RUN cd src
RUN git clone https://github.com/VictorPozoFernandez/ai4hri.git
RUN cd ..
RUN catkin make
RUN source /catkin_ws/devel/setup.bash
RUN echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc
RUN cd catkin_ws/src/ai4hri
RUN apt install python3-pip -y
RUN apt-get install portaudio19-dev python-all-dev -y
RUN pip install -r requirements.txt

RUN roslaunch ai4hri knowledge_recognition.launch
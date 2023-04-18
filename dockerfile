FROM osrf/ros:noetic-desktop-full-focal

RUN apt update
RUN apt upgrade -y
RUN apt install git -y
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc % change to export source, no need to reinitialize
RUN mkdir catkin_ws
RUN cd catkin_ws
RUN mkdir src
RUN cd src
RUN git clone https://github.com/VictorPozoFernandez/ai4hri.git
RUN cd ..
RUN %catkin make


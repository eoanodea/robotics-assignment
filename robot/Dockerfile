# Setup
FROM ros:noetic
# FROM ubuntu:14.04
ARG NOETIC_WS=/opt/ros/noetic
ARG CATKIN_WS=/root/catkin_ws

RUN apt-get update \ 
  && apt-get install -y iputils-ping \
  && apt-get install -y git \
  && rm -rf /var/lib/apt/lists/*


RUN apt update \
  && apt install -y python3-pip \
  && pip install redis

# Workspace setup
WORKDIR ${NOETIC_WS}/share

# Build workspace
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR $CATKIN_WS/src

# Copy over files
RUN mkdir pub_sub_testing
COPY . pub_sub_testing

WORKDIR $CATKIN_WS/src/pub_sub_testing

RUN chmod +x src/listener.py
RUN chmod +x launch/pub_sub_testing.launch

WORKDIR $CATKIN_WS
RUN . ${NOETIC_WS}/setup.sh && catkin_make


# Copy over kobuki msgs
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/yujinrobot/kobuki_msgs.git -b noetic

WORKDIR $CATKIN_WS
RUN . ${CATKIN_WS}/devel/setup.sh && catkin_make

# RUN echo "export ROS_MASTER_URI=http://192.168.0.107:11311" >> ~/.bashrc
# RUN echo "export ROS_HOSTNAME=192.168.0.107" >> ~/.bashrc

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
CMD ["bash", "-c", "source /root/catkin_ws/devel/setup.bash && roslaunch pub_sub_testing pub_sub_testing.launch"]


# ENTRYPOINT ["tail"]
# CMD ["-f","/dev/null"]

# Setup
FROM ros:noetic
# FROM ubuntu:14.04
ARG NOETIC_WS=/opt/ros/noetic
ARG CATKIN_WS=/root/catkin_ws

RUN apt-get update \
  && apt-get install -y git \
  && rm -rf /var/lib/apt/lists/*


# Workspace setup
WORKDIR ${NOETIC_WS}/share

# Build workspace
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR $CATKIN_WS/src

# Copy over files
RUN mkdir py_srvcli
COPY py_srvcli_ros_1 ./py_srvcli
RUN chmod +x ./py_srvcli/src/listener.py
RUN chmod +x ./py_srvcli/launch/py_srvcli.launch

WORKDIR $CATKIN_WS
RUN . ${NOETIC_WS}/setup.sh && catkin_make


# Copy over kobuki msgs
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/yujinrobot/kobuki_msgs.git -b noetic

WORKDIR $CATKIN_WS
RUN . ${CATKIN_WS}/devel/setup.sh && catkin_make

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
CMD ["bash", "-c", "source /root/catkin_ws/devel/setup.bash && roslaunch py_srvcli py_srvcli.launch"]

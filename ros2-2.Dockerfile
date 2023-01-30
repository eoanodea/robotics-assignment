FROM ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

# WORKDIR /ros2_ws

# Install Git
RUN apt-get update && apt-get install -y git

WORKDIR $OVERLAY_WS/src
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel

WORKDIR $OVERLAY_WS

RUN rosdep install -i --from-path src --rosdistro foxy -y 
RUN echo "NINA"
# RUN colcon build

# RUN source $OVERLAY_WS/install/setup.bash

# RUN . install/local_setup.bash

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

COPY py_srvcli ./src

RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN colcon build --packages-select py_srvcli
RUN . install/local_setup.bash

CMD ["ros2", "run", "py_srvcli", "service"]
# ros2 run py_srvcli service
# ros2 run py_srvcli client 2 3


# # # run launch file
# # CMD ["ros2", "launch", "demo_nodes_py", "talker_listener_python.launch.py"]

# # # ENTRYPOINT ["tail"]
# # # CMD ["-f","/dev/null"]

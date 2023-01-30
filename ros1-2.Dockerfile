FROM ros:noetic
ARG OVERLAY_WS=/opt/ros/noetic

# install ros package
RUN apt-get update && apt-get install -y \
      inetutils-ping && \
      rm -rf /var/lib/apt/lists/*


# Install Git
# RUN apt-get update && apt-get install -y git

# WORKDIR $OVERLAY_WS/src
# RUN git clone https://github.com/ros/ros_tutorials.git -b noetic-devel




# RUN rosdep install -i --from-path src --rosdistro noetic -y 
# RUN colcon build

# RUN source $OVERLAY_WS/install/setup.bash

# RUN . install/local_setup.bash

# source entrypoint setup
# ENV OVERLAY_WS $OVERLAY_WS
# RUN sed --in-place --expression \
#       '$isource "$OVERLAY_WS/install/setup.bash"' \
#       /ros_entrypoint.sh
WORKDIR $OVERLAY_WS/share
RUN mkdir py_srvcli
COPY py_srvcli_ros_1 ./py_srvcli

WORKDIR $OVERLAY_WS
RUN . ./local_setup.sh

WORKDIR $OVERLAY_WS/share/py_srvcli/py_srvcli
RUN chmod +x listener.py

CMD ["roslaunch", "py_srvcli", "py_srvcli.launch"]

# ENTRYPOINT ["tail"]
# CMD ["-f","/dev/null"]

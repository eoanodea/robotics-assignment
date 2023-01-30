FROM ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

# Install Git
RUN apt-get update && apt-get install -y git

WORKDIR $OVERLAY_WS/src
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel

WORKDIR $OVERLAY_WS

RUN rosdep install -i --from-path src --rosdistro foxy -y 

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

# To debug the container
# ENTRYPOINT ["tail"]
# CMD ["-f","/dev/null"]

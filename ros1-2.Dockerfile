FROM ros:noetic
ARG OVERLAY_WS=/opt/ros/noetic

# install ros package
RUN apt-get update && apt-get install -y \
      inetutils-ping && \
      rm -rf /var/lib/apt/lists/*

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

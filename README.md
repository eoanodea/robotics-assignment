# Robotics Assignment

A talker in ROS 2 (`py_srvcli`), a listener in ROS 1 (`py_srvcli_ros_1`) and a bridge which allows them to talk to each other

## How to run

Make sure you have docker & docker-compose installed

Run `docker-compose up`

## Controlling the Robot via SSH

First shell:

`roslaunch kobuki_node minimal.launch --screen`

Second shell:

`roslaunch kobuki_keyop safe_keyop.launch --screen`

Third shell:

`roslaunch astra_launch astra.launch`

Fourth shell:

`rqt_image_view`

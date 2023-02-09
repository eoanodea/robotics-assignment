# Robotics Assignment

A Docker stack containing `ROS1`, `ROS2` and a `Bridge` container, which can connect, communicate and control a [TURTLEBOT 2](http://kobuki.yujinrobot.com/about2), over a local WiFi network.

```mermaid
graph LR
    subgraph Robot
        A[ROS1]
    end
    A[ROS1] <---> B[ROS1 Container]
    subgraph Docker
        B[ROS1] <---> |Bridge| D[ROS2]
        D[ROS2] <---> E[Planner]
    end
```

> Deployment Diagram

Since the Physical robot's dated software can only run `ROS1`, we set up a `ROS1` container that can listen and talk to the robot, the `ROS1` container sends the information it receives to `ROS2` over a Bridge, which allows bidirectional communication between `ROS1` and `ROS2`.

The procedure of finding people and following the first recognized person:

```mermaid
stateDiagram-v2
    state human_detected_if_state <<choice>>
    state outside_center_if_state <<choice>>
    state away_robot_if_state <<choice>>
    state full_rotation_if_state <<choice>>
    state act_if_state <<choice>>

    state join_state <<join>>

    human_detected: Human Detected?
    rotate: Rotate
    full_rotation: Full Rotation
    get_id: Get ID
    get_coords: Get Coordinates
    outside_center: Outside center?
    away_robot: Away from Robot?
    adjust_rotation: Adjust Rotation
    move_forward: Move Forward
    sleep: Sleep

    [*] --> Scan

    state Scan {
        [*] --> human_detected
        human_detected --> human_detected_if_state
        human_detected_if_state --> [*] : yes
        human_detected_if_state --> rotate: no
        rotate --> human_detected
        rotate --> full_rotation
        full_rotation --> full_rotation_if_state
        full_rotation_if_state --> sleep: yes
        full_rotation_if_state --> rotate: no
        sleep --> rotate
    }

    Scan --> Detect

    state Detect {
        [*] --> get_id
        get_id --> get_id: for each person
        get_id --> get_coords: lowest ID
        get_coords --> [*]
    }

    Detect --> Act: coordinates

    state Act {
        [*] --> act_if_state: position
        act_if_state --> outside_center
        act_if_state --> away_robot
        outside_center --> outside_center_if_state
        outside_center_if_state --> adjust_rotation: yes
        adjust_rotation --> outside_center
        outside_center_if_state --> join_state: no

        away_robot --> away_robot_if_state
        away_robot_if_state --> join_state: no
        away_robot_if_state --> move_forward: yes
        join_state --> [*]
    }

    Act --> Scan
```

> State Diagram

## How to run

Make sure you have [docker & docker-compose](https://docs.docker.com/get-docker/) installed

Run `docker-compose up` (might take a while on your first run, my last fresh install took me 50 minutes 🙃)

## Controlling the Robot via SSH

First shell:

`roslaunch kobuki_node minimal.launch --screen`

Second shell:

`roslaunch kobuki_keyop safe_keyop.launch --screen`

Third shell:

`roslaunch astra_launch astra.launch`

Fourth shell:

`rqt_image_view`

Fifth shell (just for debugging):
`roslaunch pub_sub_testing pub_sub_testing.launch`

# Notes

Ros1 is a listener on another topic for only the robot is subscribed to

Topic on the robot to listen for controls made by a controller

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

Three containers should be created within a stack, with the following names:

- `assignment-ros1-1`
- `assignment-ros2-1`
- `assignment-bridge-1`

## Troubleshooting

If you want to shell into a particular container for debugging purposes, you can run the following command:

`docker exec -it [container-name] bash`

E.g.

`docker exec -it assignment-ros1-1 bash`

You may run into an issue where you cannot access the ROS CLI. In that case you should source the setup file

In ROS2: `source install/setup.bash`

In ROS1: `source devel/setup.bash`

(You shouldn't have to do this in the ROS1 container)

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

## Available Topics (On Kobuki Node)

The following topics are available when running Kobuki Node:

### Published topics:

```
 * /mobile_base/events/robot_state [kobuki_msgs/RobotStateEvent] 1 publisher
 * /mobile_base/debug/raw_data_stream [std_msgs/String] 1 publisher
 * /tf [tf2_msgs/TFMessage] 1 publisher
 * /odom [nav_msgs/Odometry] 1 publisher
 * /mobile_base/sensors/core [kobuki_msgs/SensorState] 1 publisher
 * /mobile_base/events/button [kobuki_msgs/ButtonEvent] 1 publisher
 * /mobile_base/events/power_system [kobuki_msgs/PowerSystemEvent] 1 publisher
 * /diagnostics [diagnostic_msgs/DiagnosticArray] 1 publisher
 * /mobile_base/events/digital_input [kobuki_msgs/DigitalInputEvent] 1 publisher
 * /mobile_base/events/wheel_drop [kobuki_msgs/WheelDropEvent] 1 publisher
 * /mobile_base/debug/raw_control_command [std_msgs/Int16MultiArray] 1 publisher
 * /joint_states [sensor_msgs/JointState] 1 publisher
 * /rosout [rosgraph_msgs/Log] 3 publishers
 * /mobile_base/debug/raw_data_command [std_msgs/String] 1 publisher
 * /mobile_base/sensors/imu_data_raw [sensor_msgs/Imu] 1 publisher
 * /mobile_base/sensors/dock_ir [kobuki_msgs/DockInfraRed] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /mobile_base/events/bumper [kobuki_msgs/BumperEvent] 1 publisher
 * /diagnostics_toplevel_state [diagnostic_msgs/DiagnosticStatus] 1 publisher
 * /mobile_base/controller_info [kobuki_msgs/ControllerInfo] 1 publisher
 * /mobile_base/events/cliff [kobuki_msgs/CliffEvent] 1 publisher
 * /mobile_base/sensors/imu_data [sensor_msgs/Imu] 1 publisher
 * /mobile_base_nodelet_manager/bond [bond/Status] 2 publishers
 * /mobile_base/version_info [kobuki_msgs/VersionInfo] 1 publisher
 * /diagnostics_agg [diagnostic_msgs/DiagnosticArray] 1 publisher
```

### Subscribed topics:

```
 * /mobile_base/commands/motor_power [kobuki_msgs/MotorPower] 1 subscriber
 * /mobile_base/commands/external_power [kobuki_msgs/ExternalPower] 1 subscriber
 * /mobile_base/commands/reset_odometry [std_msgs/Empty] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /mobile_base/commands/sound [kobuki_msgs/Sound] 1 subscriber
 * /mobile_base_nodelet_manager/bond [bond/Status] 2 subscribers
 * /diagnostics [diagnostic_msgs/DiagnosticArray] 1 subscriber
 * /mobile_base/commands/digital_output [kobuki_msgs/DigitalOutput] 1 subscriber
 * /mobile_base/commands/velocity [geometry_msgs/Twist] 1 subscriber
 * /mobile_base/commands/led1 [kobuki_msgs/Led] 1 subscriber
 * /mobile_base/commands/led2 [kobuki_msgs/Led] 1 subscriber
 * /mobile_base/commands/controller_info [kobuki_msgs/ControllerInfo] 1 subscriber
```

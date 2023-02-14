# 🤖 Robotics Assignment

## ✏️ Description

The aim of this project is to communicate and control a [TURTLEBOT 2](http://kobuki.yujinrobot.com/about2) over a local WiFi network. There are two parts to this project, the physical part where we utilise the Turtlebot, by using `Kobuki Node` in `ROS1`. The simulated part uses the same code but along with Gazebo Garden.

A Docker stack contains the following:

1. `master` - recieves perceptions from either a physical robot or a simulated one (`robot`), uses logic to generate commands and sends them back over a redis channel
2. `robot` - a ROS1 package used on the robot to recieve topics from Kobuki Node, and converts them and sends them over a Redis channel. (`robot` can also be used in simulated mode over Docker)
3. `redis` - used as the Redis server in the simulated version only, in the physical version, the robot has a redis server running on it.

## 🤔 How to run this code (V2)

> Make sure you have [docker & docker-compose](https://docs.docker.com/get-docker/) installed

**☁️ Simulated Version** <br/>

1. Check the `.env` file contains the right value (Should be `redis` for the simulated redis environment)

2. Uncomment everything below `# # For simulated version` in the `docker-compose.yml` file.

3. Run the master from your computer `docker-compose up` (Might take a little while on the first run)

**🤖 Physical Version** <br />

1. Check the `.env` file contains the right value (Should be `192.168.0.107` for the robot)

2. Copy over the files (From your local machine):
   `rsync -av ./robot/* ubuntu@192.168.0.107:/home/ubuntu/catkin_make_ws/src/pub_sub_testing`

3. Run the setup script (From the robot):
   `bash /home/ubuntu/catkin_ws/src/pub_sub_testing/init.sh`

4. Open up four shells in the robot and run the following:

   - `roslaunch kobuki_node minimal.launch --screen`
   - `roslaunch kobuki_keyop safe_keyop.launch --screen`
   - `roslaunch astra_launch astra.launch`
   - `roslaunch pub_sub_testing pub_sub_testing.launch`

5. Run the master from your computer `docker-compose up` (Might take a little while on the first run)

## 🪲 Troubleshooting

If you want to shell into a particular container for debugging purposes, you can run the following command:

`docker exec -it [container-name] bash`

E.g.

`docker exec -it robotics-assignment-ros1-1 bash`

If you want to test messages going from `ROS1` through the system and back, you can run (From `ROS1` container **OR** Robot)
`rostopic pub /debug/percept std_msgs/String "Hello"`

## 🏗️ Architecture (🚨 OUTDATED)

> This section is from our first iteration of the project (which we had to scrap most of it due to old operating systems)

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

## 📄 Available Topics (On Kobuki Node)

The following topics are available when running Kobuki Node:

### 📣 Published topics:

| Topic Name                                         | Message Type                          | Number of Publishers |
| -------------------------------------------------- | ------------------------------------- | -------------------- |
| /camera/depth/image                                | sensor_msgs/Image                     | 1                    |
| /camera/depth_registered/sw_registered/image_rect  | sensor_msgs/Image                     | 1                    |
| /camera/depth_registered/points                    | sensor_msgs/PointCloud2               | 1                    |
| /cmd_vel_mux/parameter_descriptions                | dynamic_reconfigure/ConfigDescription | 1                    |
| /mobile_base/events/robot_state                    | kobuki_msgs/RobotStateEvent           | 1                    |
| /mobile_base/debug/raw_data_stream                 | std_msgs/String                       | 1                    |
| /tf                                                | tf2_msgs/TFMessage                    | 5                    |
| /odom                                              | nav_msgs/Odometry                     | 1                    |
| /mobile_base/sensors/core                          | kobuki_msgs/SensorState               | 1                    |
| /camera/rgb_rectify_color/parameter_updates        | dynamic_reconfigure/Config            | 1                    |
| /cmd_vel_mux/parameter_updates                     | dynamic_reconfigure/Config            | 1                    |
| /keyop_vel_smoother/parameter_updates              | dynamic_reconfigure/Config            | 1                    |
| /mobile_base/events/button                         | kobuki_msgs/ButtonEvent               | 1                    |
| /camera/depth_rectify_depth/parameter_descriptions | dynamic_reconfigure/ConfigDescription | 1                    |
| /camera/depth_registered/sw_registered/camera_info | sensor_msgs/CameraInfo                | 1                    |
| /mobile_base/events/power_system                   | kobuki_msgs/PowerSystemEvent          | 1                    |
| /camera/depth_rectify_depth/parameter_updates      | dynamic_reconfigure/Config            | 1                    |
| /cmd_vel_mux/active                                | std_msgs/String                       | 1                    |
| /diagnostics                                       | diagnostic_msgs/DiagnosticArray       | 1                    |
| /mobile_base/events/digital_input                  | kobuki_msgs/DigitalInputEvent         | 1                    |
| /cmd_vel_mux/safety_controller                     | geometry_msgs/Twist                   | 1                    |
| /mobile_base/events/wheel_drop                     | kobuki_msgs/WheelDropEvent            | 1                    |
| /keyop_vel_smoother/parameter_descriptions         | dynamic_reconfigure/ConfigDescription | 1                    |
| /mobile_base/debug/raw_control_command             | std_msgs/Int16MultiArray              | 1                    |
| /joint_states                                      | sensor_msgs/JointState                | 1                    |
| /rosout                                            | rosgraph_msgs/Log                     | 20                   |
| /mobile_base/debug/raw_data_command                | std_msgs/String                       | 1                    |
| /mobile_base/sensors/imu_data_raw                  | sensor_msgs/Imu                       | 1                    |
| /mobile_base/sensors/dock_ir                       | kobuki_msgs/DockInfraRed              | 1                    |
| /rosout_agg                                        | rosgraph_msgs/Log                     | 1                    |
| /mobile_base/events/bumper                         | kobuki_msgs/BumperEvent               | 1                    |
| /cmd_vel_mux/keyboard_teleop                       | geometry_msgs/Twist                   | 1                    |
| /mobile_base/commands/velocity                     | geometry_msgs/Twist                   | 1                    |
| /diagnostics_toplevel_state                        | diagnostic_msgs/DiagnosticStatus      | 1                    |
| /mobile_base/controller_info                       | kobuki_msgs/ControllerInfo            | 1                    |

### 📡 Subscribed topics:

| Topic                                 | Message Type                    | Number of Subscribers |
| ------------------------------------- | ------------------------------- | --------------------- |
| /tf                                   | tf2_msgs/TFMessage              | 1                     |
| /odom                                 | nav_msgs/Odometry               | 1                     |
| /mobile_base/commands/reset_odometry  | std_msgs/Empty                  | 1                     |
| /tf_static                            | tf2_msgs/TFMessage              | 1                     |
| /diagnostics                          | diagnostic_msgs/DiagnosticArray | 1                     |
| /cmd_vel_mux/safety_controller        | geometry_msgs/Twist             | 1                     |
| /mobile_base/events/wheel_drop        | kobuki_msgs/WheelDropEvent      | 1                     |
| /mobile_base/commands/external_power  | kobuki_msgs/ExternalPower       | 1                     |
| /rosout                               | rosgraph_msgs/Log               | 1                     |
| /keyop/teleop                         | kobuki_msgs/KeyboardInput       | 1                     |
| /kobuki_safety_controller/disable     | std_msgs/Empty                  | 1                     |
| /mobile_base/commands/sound           | kobuki_msgs/Sound               | 1                     |
| /mobile_base/events/bumper            | kobuki_msgs/BumperEvent         | 1                     |
| /cmd_vel_mux/keyboard_teleop          | geometry_msgs/Twist             | 1                     |
| /mobile_base/commands/digital_output  | kobuki_msgs/DigitalOutput       | 1                     |
| /mobile_base/commands/velocity        | geometry_msgs/Twist             | 1                     |
| /mobile_base/commands/led1            | kobuki_msgs/Led                 | 1                     |
| /mobile_base/commands/led2            | kobuki_msgs/Led                 | 1                     |
| /kobuki_safety_controller/enable      | std_msgs/Empty                  | 1                     |
| /mobile_base/commands/motor_power     | kobuki_msgs/MotorPower          | 1                     |
| /mobile_base/events/cliff             | kobuki_msgs/CliffEvent          | 1                     |
| /keyop_vel_smoother/raw_cmd_vel       | geometry_msgs/Twist             | 1                     |
| /mobile_base_nodelet_manager/bond     | bond/Status                     | 5                     |
| /kobuki_safety_controller/reset       | std_msgs/Empty                  | 1                     |
| /mobile_base/commands/controller_info | kobuki_msgs/ControllerInfo      | 1                     |

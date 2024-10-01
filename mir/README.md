# Mir Hardware Helper
## 0. Package origin
This package came originally from (mir_robot)[https://github.com/dfki-ric/mir_robot].

## 1. Package overview
* `mir_description`: URDF description of the MiR robot
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_launch_sim`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files
* `mir_launch_hardware`  : Standalone launchfiles for some different tasks

## 2. Installation
### Install dependencies
```
git submodule init
git submodule update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build system to build the downloaded packages



## 3. Example - Collaborative Object Transport
Launch the object transport example launch
```
roslaunch mir_examples object_transport.launch
```

Move the "virtual_leader/base_link" to the correct position. The "virtual_leader/base_link" serves as a reference for the other robots and should ideally be in the middle between all robots. The "virtual_leader/base_link" can be moved using "rqt_robot_steer" on the "/virtual_leader/cmd_vel" topic or by publishing the desired pose on "/virtual_leader/set_pose". In this case we will set "virtual_leader/base_link" to be 1 m in front and to the side of robot mur620a using this script: 
```
roslaunch virtual_leader set_leader_pose.launch relative_pose:=[1.0,1.0,0.0] robot_pose_topic:=/mur620a/mir_pose_stamped_simple
```

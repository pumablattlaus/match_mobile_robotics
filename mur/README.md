# How to use perform a cooperative handling sequence

## 📝 Table of Contents
- [Setup Simulation](#setup_sim)


## 1. Setup Simulation <a name = "setup_sim"></a>

1. Launch a world 
``` 
roslaunch match_gazebo big_square.launch 
``` 
2. Spawn MuRs with different namespaces
``` 
roslaunch mur_examples multi_mur620_handling.launch
``` 
Make sure move_group is successfully loaded. Maybe you have to re-spawn the robots with gazebo running (delete them in gazebo and ctrl+c the launch file terminal). If move_group is successfull, the manipulators should all move to their home position. If everything has been started correctly, the robots should now look like in the picture.

<!-- add image -->
![Alt text](mur_documentation/murs_spawned.png?raw=true "Four MuRs spawned successfully")

3. Move the robots to their handling position
``` 
rosrun mur_examples move_all_URs_to_pose.py
``` 
The robots are moved using move_group. This may take a while.
![Alt text](mur_documentation/MuRs_in_handling_pose.png?raw=true "All MuRs in handling pose")

4. Turn on controllers
``` 
roslaunch mur_control multi_decentralized_admittance_controller_sim.launch
``` 
Launch eight admittance controller nodes simulanously. Each robot receives its relative target pose through the controller launch.
Check in RViz if the current pose roughly matches the target pose. It should look something like this:
![Alt text](mur_documentation/RVIZ_ready.png?raw=true "All admittance controllers are running. Robots are almost in target pose.")

5. Switch the robot to twist control
```
rosrun mur_examples switch_URs_to_twist_control.py
```
So far the official UR ROS package does not provide a twist controller in gazebo (only in hardware).
Instead we use a custom twist controller based on the inverse differential kinematics and the joint_group_vel controller.

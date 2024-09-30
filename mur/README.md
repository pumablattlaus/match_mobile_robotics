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
Make sure move_group is successfully loaded. Maybe you have to re-spawn the robots with gazebo running (delete them in gazebo and ctrl+c the launch file terminal). If move_group is successfull, the manipulators should all move to their home position.
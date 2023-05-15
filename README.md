# Dependency
OCS2(Hpp-fcl,ocs2_robotic_assets): 
Code: https://github.com/P1terQ/ocs2
Instruction: https://github.com/leggedrobotics/ocs2

Elevation_Mapping_Cupy: 
Code: https://github.com/P1terQ/elevation_mapping_cupy/tree/labpeter 
Instruction: https://github.com/leggedrobotics/elevation_mapping_cupy

realsense_ros_gazebo:
Code: https://github.com/P1terQ/realsense_ros_gazebo/tree/labpeter
Instruction: https://github.com/nilseuropa/realsense_ros_gazebo


# Command
1. Launch gazebo: roslaunch legged_unitree_description empty_world.launch
2. Launch controller: roslaunch legged_controllers load_controller.launch
3. Launch mapping: roslaunch elevation_mapping_cupy a1_segmentation.launch

# Robot
Now this repo support unitree A1 and unitree aliengo. To play with different robots, user should modify the robot and parameters in load_controller.launch,  empty_world.launch, SwitchedModelReferenceManager.cpp

# Experiment in Reality
User should modify IP address in udp.h
To compensate for IMU drift or model error, modify zyxOffset_ in StateEstimateBase.cpp

# Robot Walking Upstairs
https://user-images.githubusercontent.com/21256355/192135828-8fa7d9bb-9b4d-41f9-907a-68d34e6809d8.mp4
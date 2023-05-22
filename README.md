# Dependency
OCS2(Hpp-fcl,ocs2_robotic_assets): 
Code: https://github.com/P1terQ/ocs2 -b master
Original Repo: https://github.com/leggedrobotics/ocs2

Quad_Controller
Code: https://github.com/P1terQ/quad_controller -b master
Original Repo: https://github.com/qiayuanliao/legged_control

Elevation_Mapping_Cupy: 
Code: https://github.com/P1terQ/elevation_mapping_cupy -b master
Original Repo: https://github.com/leggedrobotics/elevation_mapping_cupy

realsense_ros_gazebo:
Code: https://github.com/P1terQ/realsense_ros_gazebo -b master
Original Repo: https://github.com/nilseuropa/realsense_ros_gazebo


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
https://github.com/P1terQ/quad_controller/assets/90088571/92b1b6c0-7170-44cf-90bf-b88acee23147

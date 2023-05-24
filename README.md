# Dependency
User should clone all the following packages into one ros working space

OCS2(note: OCS2 also needs Hpp-fcl,ocs2_robotic_assets,punocchio): 
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

# Installation
1. Build OCS2 by 'catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization'
2. Build Elevation_Mapping_Cupy by 'catkin build elevation_mapping_cupy' and 'catkin build convex_plane_decomposition_ros'
3. Build realsense_ros_gazebo by 'catkin build realsense_ros_gazebo'
4. Build Quad_Controller by 'legged_controllers legged_unitree_description legged_gazebo legged_unitree_hw'

# Manul
1. Launch gazebo simulation by 'roslaunch legged_unitree_description empty_world.launch'
2. Launch controller by 'roslaunch legged_controllers load_controller.launch' and use controller_manager to start the controller. Note: User can specify whether to use perceptive controller by modifying 'if_perceptive'
3. Launch mapping by 'roslaunch elevation_mapping_cupy a1_segmentation.launch' and wait until the segmentation appears in RVIZ
4. Input the gait in the terminal of controller and Input target by '/cmd_vel' or '/move_base_simple/goal' or '/dummy_navigator_PTP/rel_goal' or '/dummy_navigator_PTP/abs_goal'

# Robot
Now this repo support unitree A1 and unitree aliengo. To play with different robots, user should modify the robot and parameters in load_controller.launch,  empty_world.launch, SwitchedModelReferenceManager.cpp

# Experiment in Reality
User should modify IP address in udp.h
To compensate for IMU drift or model error, modify zyxOffset_ in StateEstimateBase.cpp

# Robot Walking Upstairs
https://github.com/P1terQ/quad_controller/assets/90088571/92b1b6c0-7170-44cf-90bf-b88acee23147

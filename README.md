# V-RPC : Vision-Based Reactive Planning and Control of Quadruped Robots in Unstructured Dynamic Environments 
Website: https://sites.google.com/view/v-rpc <br>
Paper: https://arxiv.org/pdf/2307.10243.pdf <br>

# Dependency Installation
Note: Users should install all the following dependencies into one ros work space.

OCS2: <br>
`git clone https://github.com/P1terQ/ocs2 -b master` <br>
Original Repo: https://github.com/leggedrobotics/ocs2  <br>
(note: OCS2 also needs Hpp-fcl,ocs2_robotic_assets,pinocchio. Users should follow the installation manual in https://leggedrobotics.github.io/ocs2/installation.html)

Quad_Controller: <br>
`git clone https://github.com/P1terQ/quad_controller -b master` <br>
Original Repo: https://github.com/qiayuanliao/legged_control

Elevation_Mapping_Cupy: <br>
`https://github.com/P1terQ/elevation_mapping_cupy -b master` <br>
Original Repo: https://github.com/leggedrobotics/elevation_mapping_cupy

realsense_ros_gazebo: <br>
`https://github.com/P1terQ/realsense_ros_gazebo -b master` <br>
Original Repo: https://github.com/nilseuropa/realsense_ros_gazebo

# Build
1. Build OCS2 by 'catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization'
2. Build Elevation_Mapping_Cupy by 'catkin build elevation_mapping_cupy' and 'catkin build convex_plane_decomposition_ros'
3. Build realsense_ros_gazebo by 'catkin build realsense_ros_gazebo'
4. Build Quad_Controller by 'legged_controllers legged_unitree_description legged_gazebo legged_unitree_hw'


# Usage
1. Launch gazebo simulation: <br> `roslaunch legged_unitree_description empty_world.launch` 
2. Launch controller: <br> `roslaunch legged_controllers load_controller.launch`  <br> Use ros controller_manager plugin to start the controller. Note: User can specify whether to use perceptive controller by modifying 'if_perceptive' flag in the launch file.
3. Launch mapping: <br> `roslaunch elevation_mapping_cupy a1_segmentation.launch`
4. Input the gait in the terminal of controller and Input target by `/cmd_vel`, `/move_base_simple/goal`, `/dummy_navigator_PTP/rel_goal`, and `/dummy_navigator_PTP/abs_goal` (The above are ros topics.)

# Robot
Now this repo support unitree A1 and unitree aliengo. To play with different robots, user should modify the robot and parameters in `load_controller.launch`,  `empty_world.launch`, `SwitchedModelReferenceManager.cpp`.

# Experiment in Reality
Users should modify IP address in `udp.h` to your robot's PC IP address.
To compensate for IMU drift or model error, modify the value`zyxOffset_` in `StateEstimateBase.cpp`.

# Robot Walking Stairs 
https://github.com/P1terQ/quad_controller/assets/90088571/92b1b6c0-7170-44cf-90bf-b88acee23147

``

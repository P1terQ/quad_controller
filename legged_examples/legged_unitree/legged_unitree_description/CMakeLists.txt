cmake_minimum_required(VERSION 3.10)
project(legged_unitree_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        controller_manager
        genmsg
        joint_state_controller
        robot_state_publisher
        roscpp
        gazebo_ros
        std_msgs
        tf
        geometry_msgs
        # unitree_legged_msgs
        )

find_package(gazebo REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        # unitree_legged_msgs 
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#############
## Install ##
#############

# Mark resource files for installation
install(
        DIRECTORY meshes urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


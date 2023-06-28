cmake_minimum_required(VERSION 3.10)
project(reactive_gait)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        darknet_ros_msgs
        ocs2_msgs
        )

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        darknet_ros_msgs
        ocs2_msgs
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}
        src/reactive_gait_node.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


#########################
###   CLANG TOOLING   ###
#########################
# find_package(cmake_clang_tools QUIET)
# if (cmake_clang_tools_FOUND)
#     message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
#     add_clang_tooling(
#             TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
#             SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
#             CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
#             CF_WERROR
#     )
# endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############


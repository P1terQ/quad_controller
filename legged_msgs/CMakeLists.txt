cmake_minimum_required(VERSION 3.0.2)
project(legged_msgs)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  lowstate.msg
  leggedreference.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
  
#   CheckSafety.srv
#   Initialize.srv
#   ConvexAppro.srv
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
)

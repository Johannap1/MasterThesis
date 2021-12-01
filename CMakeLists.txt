
#Check minimum requirement for cmake version
cmake_minimum_required(VERSION 3.0.2)
#Define the project
project(simulation_test)

#Import the required packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs mavros_msgs)
catkin_package()

#Include Directories
include_directories(include ${catkin_INCLUDE_DIRS})

#Create executable simple_control
add_executable(offboard_control_att src/offb_node_att.cpp src/pid.cpp src/input_generator.cpp)
add_executable(offboard_control_vel src/offb_node_vel.cpp)

#Use appropriate Library files
target_link_libraries(offboard_control_att ${catkin_LIBRARIES})
target_link_libraries(offboard_control_vel ${catkin_LIBRARIES})

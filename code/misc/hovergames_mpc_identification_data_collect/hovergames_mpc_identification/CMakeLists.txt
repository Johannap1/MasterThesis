cmake_minimum_required(VERSION 3.0.2)
project(hovergames_mpc_identification)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/MPCIdentificationGazebo.cfg
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}
 LIBRARIES hovergames_mpc_identification
 # no CATKIN_DEPENDS, because this package does not contain header files that will be used by other packages
)

include_directories(include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mpc_identification_exp src/mpc_identification_exp.cpp)

add_library(data_collector src/data_collector.cpp)

add_library(pid_control src/pid_control.cpp)
add_library(lqr_control src/lqr_control.cpp)

add_library(simple_control src/simple_control.cpp)

add_library(input_generator src/input_generator.cpp)

add_dependencies(mpc_identification_exp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mpc_identification_exp_node src/mpc_identification_exp_node.cpp)

add_executable(data_collector_node src/data_collector_node.cpp)

#Add dependencies for dynamic header files to get created before
add_dependencies(mpc_identification_exp_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(mpc_identification_exp ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(data_collector_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(data_collector ${${PROJECT_NAME}_EXPORTED_TARGETS})

#Use appropriate Library files
target_link_libraries(mpc_identification_exp_node
  mpc_identification_exp
  pid_control
  lqr_control
  input_generator
  simple_control
  ${catkin_LIBRARIES}
)

target_link_libraries(data_collector_node
  data_collector
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(raksha)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  geometry_msgs
  gazebo_ros
)
find_package(PCL REQUIRED)
find_package(gazebo REQUIRED)
catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs
)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})



# add_executable(pointCloud src/pointCloud.cpp)
add_executable(depthSensor src/depthSensor.cpp)
target_link_libraries(depthSensor ${catkin_LIBRARIES})

add_library(counterAttack src/counterAttack.cpp)
target_link_libraries(counterAttack ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
cmake_minimum_required(VERSION 2.8.3)
project(experiment_launcher)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

catkin_package(

)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gazebo_ground_truth src/gazebo_ground_truth.cpp)
target_link_libraries(gazebo_ground_truth ${catkin_LIBRARIES})

add_executable(floor5_robot1 src/floor5_robot1.cpp)
target_link_libraries(floor5_robot1 ${catkin_LIBRARIES})

add_executable(floor5_robot2 src/floor5_robot2.cpp)
target_link_libraries(floor5_robot2 ${catkin_LIBRARIES})
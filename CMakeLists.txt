cmake_minimum_required(VERSION 2.8.3)
project(experiment_launcher)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

# Find LibXml2 library
find_package(LibXml2 2.6.0 REQUIRED)

# Find Armadillo library
find_package(Armadillo REQUIRED)

catkin_package(

)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gazebo_ground_truth src/gazebo_ground_truth.cpp)
target_link_libraries(gazebo_ground_truth ${catkin_LIBRARIES})

add_executable(floor5_robot1 src/floor5_robot1.cpp)
target_link_libraries(floor5_robot1 ${catkin_LIBRARIES})

add_executable(floor5_robot2 src/floor5_robot2.cpp)
target_link_libraries(floor5_robot2 ${catkin_LIBRARIES})
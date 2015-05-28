cmake_minimum_required(VERSION 2.8.3)
project(robotnik_gyro)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    tf
    nav_msgs 
    std_msgs
    std_srvs
    geometry_msgs 
    sensor_msgs 
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Battery.msg  
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs std_msgs geometry_msgs sensor_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES summit_xl_robot_control
  CATKIN_DEPENDS 
    roscpp 
    tf
    nav_msgs 
    std_msgs
    geometry_msgs 
    sensor_msgs 
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(robotnik_gyro src/robotnik_gyro.cpp src/SerialDevice.cc)
add_dependencies(robotnik_gyro robotnik_gyro_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_gyro ${catkin_LIBRARIES} )

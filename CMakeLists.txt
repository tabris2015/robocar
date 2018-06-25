cmake_minimum_required(VERSION 2.8.3)
project(robocar)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
  turtlesim
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robocar
#  CATKIN_DEPENDS joy roscpp rospy std_msgs turtlesim
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  "include/robocar"
  ${catkin_INCLUDE_DIRS}
)

# joy teleop

add_executable(joy_teleop 
              src/joy_teleop.cpp)

target_link_libraries(joy_teleop 
              ${catkin_LIBRARIES})

add_dependencies(joy_teleop robocar_generate_messages_cpp)

# i2c sensors
add_executable(i2c_hub 
              src/i2c_hub.cpp
              src/motor_driver_i2c.cpp
              src/VL53L0X.cpp
              src/I2Cdev.cpp
              )

target_link_libraries(i2c_hub 
              ${catkin_LIBRARIES}
              RTIMULib    # libreria para la IMU
              )


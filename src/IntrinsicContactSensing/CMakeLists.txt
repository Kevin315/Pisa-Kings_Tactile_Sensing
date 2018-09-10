cmake_minimum_required(VERSION 2.8)
project( ics )

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(Boost 1.6 REQUIRED COMPONENTS system thread atomic timer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sr_grasp_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  roslib
  cmake_modules
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sr_grasp_tactile
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs
  DEPENDS eigen
)

find_package(Boost 1.6 REQUIRED COMPONENTS system thread atomic timer)
find_package(Eigen REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories( 
  ${catkin_INCLUDE_DIRS}
  include
  ${EIGEN_INCLUDE_DIRS}
)
add_library(FTSensors STATIC src/FTSensors.cpp)
add_library(ITSurfaces STATIC src/ITSurfaces.cpp)
add_library(IntrinsicContactSensing STATIC src/IntrinsicContactSensing.cpp)

target_link_libraries(IntrinsicContactSensing FTSensors ITSurfaces ${Boost_LIBRARIES})

#add_executable( ATI_NetBox_FT_Sensor ${PROJECT_SOURCE_DIR}/src/ATI_NetBox_FT_Sensor.cpp )
#add_dependencies( ATI_NetBox_FT_Sensor 
#	FTSensors
#)
#target_link_libraries(ATI_NetBox_FT_Sensor ${catkin_LIBRARIES})
#target_link_libraries(ATI_NetBox_FT_Sensor ${boost_LIBRARIES})

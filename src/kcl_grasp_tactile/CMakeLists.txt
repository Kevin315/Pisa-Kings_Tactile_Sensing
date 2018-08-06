cmake_minimum_required(VERSION 2.8.3)
project(kcl_grasp_tactile)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kcl_grasp_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  roslib
  cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   kcl_grasp_msgs   std_msgs   visualization_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kcl_grasp_tactile
  CATKIN_DEPENDS roscpp kcl_grasp_msgs std_msgs visualization_msgs
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${EIGEN_INCLUDE_DIRS}

)

link_directories(
    lib
)
## Declare a cpp library
# add_library(kcl_grasp_tactile
#   src/${PROJECT_NAME}/kcl_grasp_tactile.cpp
# )

## Declare a cpp executable
 add_executable(daq_sensor src/ftrt.c src/ftconfig.c src/node.c src/dom.c src/xmlparse.c src/stack.c src/xmltok.c src/expatls.c src/daq_sensor.cpp)
target_link_libraries(daq_sensor comedi rt expat ${catkin_LIBRARIES})
add_dependencies(daq_sensor ${catkin_EXPORTED_TARGETS} ${catkin_LIBRARIES} kcl_grasp_msgs_generate_messages_cpp)

add_executable(contloc src/contloc.cpp)
add_dependencies(contloc ${catkin_EXPORTED_TARGETS})
add_dependencies(contloc kcl_grasp_msgs_generate_messages_cpp)
get_property(LIB64 GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS)
if (${LIB64} STREQUAL "TRUE")
target_link_libraries(contloc ${catkin_LIBRARIES} levmar lapack blas)
else()
target_link_libraries(contloc ${catkin_LIBRARIES} levmar32 lapack blas)
endif()

#add_executable(contloc_net src/contloc_net.cpp)
#add_dependencies(contloc_net ${catkin_EXPORTED_TARGETS})
#add_dependencies(contloc_net kcl_grasp_msgs_generate_messages_cpp)
#target_link_libraries(contloc_net ${catkin_LIBRARIES})
#if (${LIB64} STREQUAL "TRUE")
#target_link_libraries(contloc_net ${catkin_LIBRARIES} levmar lapack blas)
#else()
#target_link_libraries(contloc_net ${catkin_LIBRARIES} levmar32 lapack blas)
#endif()


add_executable(visualiser src/visualiser_new.cpp)
add_dependencies(visualiser ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualiser ${catkin_LIBRARIES} GL GLU GLEW glut)
add_dependencies(visualiser kcl_grasp_msgs_generate_messages_cpp)

add_executable(marker_viz src/marker_viz.cpp)
add_dependencies(marker_viz ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_viz ${catkin_LIBRARIES})
add_dependencies(marker_viz kcl_grasp_msgs_generate_messages_cpp)


#add_executable(pst_kcl_sensor src/pst_kcl_sensor.cpp)
#add_dependencies(pst_kcl_sensor ${catkin_EXPORTED_TARGETS})
#target_link_libraries(pst_kcl_sensor ${catkin_LIBRARIES})


#add_executable(force_only src/force_only.cpp)
#add_dependencies(force_only ${catkin_EXPORTED_TARGETS})
#target_link_libraries(force_only ${catkin_LIBRARIES})


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(kcl_grasp_tactile_node kcl_grasp_tactile_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(kcl_grasp_tactile_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS kcl_grasp_tactile kcl_grasp_tactile_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kcl_grasp_tactile.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

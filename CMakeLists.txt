cmake_minimum_required(VERSION 2.8.3)
project(icarus_rover_rc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs

  #usb_cam
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
find_package(message_generation)


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
##     * add a run_depend tag for=message_runtime"
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
add_message_files(FILES RC.msg Attitude.msg Control.msg Num.msg State.msg VFR_HUD.msg ICARUS_Diagnostic.msg)
#   ICARUS_Target_Status.msg
#   ICARUS_Probe_Status.msg
#   ICARUS_Probe_Command.msg
   #ICARUS_Diagnostic.msg


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
#generate_messages(
#  DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
 #)
generate_messages()
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
   INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
 #add_library(Evolution src/Evolution.cpp)

## Declare a cpp executable
#add_executable(acquire_train_images src/acquire_train_images.cpp)
#add_executable(pick_targets_from_train_images src/pick_targets_from_train_images.cpp)
#add_executable(tune_targeting src/tune_targeting.cpp)


add_executable(Motion_Controller_Node src/Motion_Controller_Node.cpp)
add_executable(Sonic_Controller_Node src/Sonic_Controller_Node.cpp)
add_executable(Diagnostic_Node src/Diagnostic_Node.cpp)
add_executable(Mapping_Node src/Mapping_Node.cpp)
#add_executable(Learning_Node src/Evolution_Node.cpp)

#Obsolete Nodes
#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#add_executable(Targeting_Node src/Targeting_Node.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(icarus_rover_rc_node icarus_rover_rc_generate_messages_cpp)
#add_dependencies(Motion_Controller_Node ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
# target_link_libraries(icarus_rover_rc_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(acquire_train_images 
#	${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#	${PROJECT_SOURCE_DIR}
#)

#target_link_libraries(pick_targets_from_train_images 
#	${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#	#${PROJECT_SOURCE_DIR}
#)

#target_link_libraries(tune_targeting
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
  #${PROJECT_SOURCE_DIR}
#)
#target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

#target_link_libraries(Targeting_Node
  
#  ${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#	#${PROJECT_SOURCE_DIR}
#)		

target_link_libraries(Motion_Controller_Node
  ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES} 
)		
target_link_libraries(Sonic_Controller_Node
  ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES} 
)
target_link_libraries(Diagnostic_Node
  ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES} 
)
target_link_libraries(Mapping_Node
 ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#target_link_libraries(Evolution_Node
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${roscpp_LIBRARIES}
#  Evolution
#)
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
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
## Mark executables and/or libraries for installation
# install(TARGETS icarus_rover_rc icarus_rover_rc_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_icarus_rover_rc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

##Print out all cmake variables
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

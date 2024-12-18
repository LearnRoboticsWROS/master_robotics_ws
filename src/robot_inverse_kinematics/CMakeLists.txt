cmake_minimum_required(VERSION 3.0.2)
project(robot_inverse_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  frame_transform
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_inverse_kinematics
#  CATKIN_DEPENDS interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pcl_conversions pcl_ros pluginlib roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robot_inverse_kinematics.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/robot_inverse_kinematics_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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

add_executable(test_ik src/test_ik.cpp)
add_dependencies(test_ik ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_ik ${catkin_LIBRARIES})

add_executable(test_ik_collision_objects src/test_ik_collision_objects.cpp)
add_dependencies(test_ik_collision_objects ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_ik_collision_objects ${catkin_LIBRARIES})

add_executable(pick_place src/pick_place.cpp)
add_dependencies(pick_place ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place ${catkin_LIBRARIES})

add_executable(pick_place_class src/pick_place_class.cpp)
add_dependencies(pick_place_class ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place_class ${catkin_LIBRARIES})

add_executable(ur5_pick_place_class src/pick_place_class.cpp)
add_dependencies(ur5_pick_place_class ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur5_pick_place_class ${catkin_LIBRARIES})

add_executable(test_module src/test_module.cpp)
add_dependencies(test_module ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_module ${catkin_LIBRARIES})

add_executable(pick_place_2D_camera src/pick_place_2D_camera.cpp)
add_dependencies(pick_place_2D_camera ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place_2D_camera ${catkin_LIBRARIES})

add_executable(pick_place_grasp_2D_camera src/pick_place_grasp_2D_camera.cpp)
add_dependencies(pick_place_grasp_2D_camera ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place_grasp_2D_camera ${catkin_LIBRARIES})

add_executable(pick_place_ur5_robotiq src/pick_place_ur5_robotiq.cpp)
add_dependencies(pick_place_ur5_robotiq ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place_ur5_robotiq ${catkin_LIBRARIES})

add_executable(pick_place_ur5_robotiq_3D_camera src/pick_place_ur5_robotiq_3D_camera.cpp)
add_dependencies(pick_place_ur5_robotiq_3D_camera ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place_ur5_robotiq_3D_camera ${catkin_LIBRARIES})

add_executable(repeatable_trajectory src/repeatable_trajectory.cpp)
add_dependencies(repeatable_trajectory ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(repeatable_trajectory ${catkin_LIBRARIES})

add_executable(draw_circle src/draw_circle.cpp)
add_dependencies(draw_circle ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(draw_circle ${catkin_LIBRARIES})

add_executable(test_module2 src/test_module2.cpp)
add_dependencies(test_module2 ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_module2 ${catkin_LIBRARIES})

add_executable(waypoint_saver src/waypoint_saver.cpp)
add_dependencies(waypoint_saver ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_saver ${catkin_LIBRARIES})

add_executable(waypoint_executer src/waypoint_executer.cpp)
add_dependencies(waypoint_executer ${cobot_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_executer ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_inverse_kinematics.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.4.6)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation std_srvs geometry_msgs sensor_msgs tf)

# Set the project parameters
set(PROJECT_DESCRIPTION "Making HRP-2 ride the Ergocycle")
set(PROJECT_NAME hrp2_bike)
set(PROJECT_URL "")

# Includes the relevant cmake scripts
include(cmake/base.cmake)
include(cmake/ros.cmake)
INCLUDE(cmake/eigen.cmake)
include(cmake/GNUInstallDirs.cmake)
include(cmake/python.cmake)

# Initialize project
SETUP_PROJECT()
SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("sot-application")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_service_files(
   FILES
   Input_from_user.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/cycling.py
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/dynamic_graph/sot/application #Put your own path to the installation directory
  )

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/__init__.py
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/dynamic_graph/sot/application #Put your own path to the installation directory
  )


INSTALL(DIRECTORY launch rviz_cfg scripts
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME} #Put your own path to the installation directory
  )

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/)

install(FILES scripts/launcher 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/ 
  PERMISSIONS OWNER_READ OWNER_EXECUTE OWNER_WRITE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
SETUP_PROJECT_FINALIZE()


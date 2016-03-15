cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()
set(POD_NAME valkyrie_translator)

project(valkyrie_translator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pluginlib
  roslint
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES LCM2ROSControl JointPositionGoalController
  CATKIN_DEPENDS roscpp std_msgs hardware_interface controller_interface joint_limits_interface
  DEPENDS system_lib pluginlib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

######################################################
add_library(LCM2ROSControl src/LCM2ROSControl.cpp)
target_link_libraries(LCM2ROSControl ${catkin_LIBRARIES} )
pods_use_pkg_config_packages(LCM2ROSControl lcm drc-lcmtypes lcmtypes_bot2-core)


#############
## Install ##
#############

install(TARGETS LCM2ROSControl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
install(FILES LCM2ROSControl.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslint_cpp()

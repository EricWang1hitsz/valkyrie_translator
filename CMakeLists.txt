cmake_minimum_required(VERSION 2.8.12)

set(ENV{PKG_CONFIG_PATH} $ENV{DISTRO_BASE}/build/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

project(valkyrie_translator)
set(POD_NAME valkyrie_translator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pluginlib
  roslint
  pods
)

catkin_package(
  LIBRARIES LCM2ROSControl JointPositionGoalController JointStatePublisher
  CATKIN_DEPENDS roscpp std_msgs hardware_interface controller_interface joint_limits_interface pods
  DEPENDS pluginlib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(--std=c++11)

######################################################
add_library(LCM2ROSControl src/LCM2ROSControl.cpp)
target_link_libraries(LCM2ROSControl ${catkin_LIBRARIES} )
pods_use_pkg_config_packages(LCM2ROSControl lcm lcmtypes_bot2-core lcmtypes_drc_lcmtypes)

add_library(JointPositionGoalController src/JointPositionGoalController.cpp)
target_link_libraries(JointPositionGoalController ${catkin_LIBRARIES})
pods_use_pkg_config_packages(JointPositionGoalController lcm lcmtypes_bot2-core)

add_library(JointStatePublisher src/JointStatePublisher.cpp)
target_link_libraries(JointStatePublisher ${catkin_LIBRARIES})
pods_use_pkg_config_packages(JointStatePublisher lcm lcmtypes_bot2-core)


#############
## Install ##
#############

install(TARGETS LCM2ROSControl JointPositionGoalController JointStatePublisher
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
install(FILES LCM2ROSControl.xml JointPositionGoalController.xml JointStatePublisher.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(ROSLINT_CPP_OPTS "--filter=-whitespace/line_length,-runtime/references,-runtime/indentation_namespace,-whitespace/braces,-readability/todo")

roslint_cpp(src/JointPositionGoalController.cpp src/JointStatePublisher.cpp)

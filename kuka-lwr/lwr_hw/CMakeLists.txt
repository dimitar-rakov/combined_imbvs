cmake_minimum_required(VERSION 2.8.3)
project(lwr_hw)


add_definitions (-fpermissive -std=c++11)
set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )




#a dummy_target so qtcreator have access to all files
######################################################################### 
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

#########################################################################



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  roscpp
  tf
  urdf
  cmake_modules
  pluginlib
  kdl_parser
  transmission_interface
  gazebo_ros
  std_msgs
  lwr_hw_interfaces
)




SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)



catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    controller_interface
    controller_manager
    hardware_interface
    realtime_tools
    joint_limits_interface
    roscpp
    tf
    urdf
    cmake_modules
    pluginlib
    kdl_parser
    transmission_interface
    gazebo_ros
    std_msgs
    lwr_hw_interfaces
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} lwr_hw_gazebo_plugin friremote
)


##############################################################################
## Build ##
##############################################################################



##############################################################################
## LWR HW BASE CLASS TO RULE'EM ALL
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/lwr_hw.h
  src/lwr_hw/lwr_hw.cpp
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


##############################################################################
# lwr hw fri class

# REAL KUKA FRI library
add_library(friremote
  src/fri/friremote.cpp
  src/fri/friudp.cpp
  src/fri/friudp.cpp
)

add_library(lwr_hw_fri
  src/lwr_hw/lwr_hw_fri.cpp
)

add_dependencies(lwr_hw_fri
  friremote
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(lwr_hw_fri
  friremote
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



##############################################################################
# lwr hw fri node
add_executable(lwr_hw_fri_node
  nodes/lwr_hw_fri_node.cpp
)

add_dependencies(lwr_hw_fri_node
  lwr_hw_fri
)
target_link_libraries(lwr_hw_fri_node
  lwr_hw_fri
)


##############################################################################
## REAL  Standford FRILibrary support in PROGRESS
message(WARNING "ATTENTION: This project supports the fork of the Standford
library found in https://github.com/iocroblab/fril, and install the libraries
in your catkin workspace, not in the system")

include(ExternalProject)
ExternalProject_Add(FRILibrary
  GIT_REPOSITORY https://github.com/iocroblab/fril.git
  GIT_TAG a10695fad32ba8315817c42020e2dbc66f4505dd
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND cmake -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  INSTALL_COMMAND make install
)

##############################################################################
# lwr hw fril class
include_directories(${CATKIN_DEVEL_PREFIX}/include/fril)
link_directories(${CATKIN_DEVEL_PREFIX}/lib)
add_library(lwr_hw_fril
  src/lwr_hw/lwr_hw_fril.cpp
)

add_dependencies(lwr_hw_fril
  FRILibrary
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(lwr_hw_fril
   fril
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
##############################################################################
# lwr hw fril node
include_directories(${CATKIN_DEVEL_PREFIX}/include/fril)
link_directories(${CATKIN_DEVEL_PREFIX}/lib)

add_executable(lwr_hw_fril_node
  nodes/lwr_hw_fril_node.cpp
)

add_dependencies(lwr_hw_fril_node
  FRILibrary
  ${PROJECT_NAME} 
  lwr_hw_fril
)

target_link_libraries(lwr_hw_fril_node
  fril
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  lwr_hw_fril
)


##############################################################################
## SIMULATION ## lwr hw gazebo class

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(lwr_hw_gazebo 
  src/lwr_hw/lwr_hw_gazebo.cpp
)
add_dependencies(lwr_hw_gazebo
  ${PROJECT_NAME}
)
target_link_libraries(lwr_hw_gazebo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


##############################################################################
## SIMULATION # lwr hw sim gazebo plugin

add_library(lwr_hw_gazebo_plugin
  nodes/lwr_hw_gazebo_plugin.cpp
)
add_dependencies(lwr_hw_gazebo_plugin
  lwr_hw_gazebo
  ${PROJECT_NAME}

)
target_link_libraries(lwr_hw_gazebo_plugin
  lwr_hw_gazebo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

##############################################################################
## INSTALLATION
##############################################################################
install(TARGETS ${PROJECT_NAME} lwr_hw_gazebo_plugin friremote
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS lwr_hw_fri_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

cmake_minimum_required(VERSION 2.8.3)
project(imbvs_with_obstacles_avoidance)
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

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  visualization_msgs
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  tf
  tf_conversions
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
   FILES
   PoseRPY.msg
   RPY.msg
   VisualServoing.msg
   VisFeature.msg
   ObsData.msg
)

 add_service_files(
   DIRECTORY srv
   FILES
   SetGama.srv
   SetTF.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    control_toolbox
    realtime_tools
    urdf
    kdl_parser
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/imbvs_with_obstacles_avoidance.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Declare a C++ executable
 add_executable(${PROJECT_NAME}_node nodes/imbvs_with_obstacles_avoidance_node.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)


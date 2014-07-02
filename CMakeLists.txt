cmake_minimum_required(VERSION 2.8.3)
project(transform_broadcast)

set(catkin_DIR /opt/ros/groovy/share/catkin/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp serial std_msgs sensor_msgs geometry_msgs tf)


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES transform_broadcast
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
#link_directories()


set(transform_broadcast_SRCS 
    src/transform_broadcast_node.cpp
)


## Declare a cpp library
 add_library(${PROJECT_NAME}  ${transform_broadcast_SRCS})

add_dependencies(transform_broadcast ${PROJECT_NAME})
add_executable(transform_broadcast_node src/transform_broadcast_node.cpp)
target_link_libraries(transform_broadcast_node ${catkin_LIBRARIES})

## Add dependencies to the executable
## add_dependencies(ardu_pilot_node ${PROJECT_NAME})
# add_dependencies(ardu_pilot_listener ${PROJECT_NAME})
# add_dependencies(ardu_pilot_node ardu_pilot_gencpp)
# add_dependencies(ardu_pilot_listener ardu_pilot_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ardu_pilot_node ${catkin_LIBRARIES})
# target_link_libraries(ardu_pilot_listener ${catkin_LIBRARIES})
## Declare a cpp executable
# add_executable(ardu_pilot_node src/ardu_pilot_node.cpp)
# add_executable(ardu_pilot_listener src/ardu_pilot_listener.cpp)



#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ardu_pilot ardu_pilot_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ardu_pilot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

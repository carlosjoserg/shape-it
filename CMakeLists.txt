cmake_minimum_required(VERSION 2.8.3)
project(shape_it)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
	roscpp
	pcl_ros
	actionlib_msgs
	sensor_msgs
	visualization_msgs
	geometry_msgs
)

find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

add_message_files(
  FILES
  EncodedCloud.msg
  EncodingStats.msg
)
add_action_files(
	FILES
	CloudEncoder.action
)

generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
	sensor_msgs
	visualization_msgs
	geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES shape_it
#  CATKIN_DEPENDS other_catkin_pkg
	DEPENDS roscpp pcl_ros actionlib_msgs sensor_msgs visualization_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
	${CGAL_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(encoder
	src/encoder.cpp
)
target_link_libraries( encoder
	${catkin_LIBRARIES}
	${CGAL_LIBRARIES}
)

add_executable(shapelizer
	src/shapelizer.cpp
)
target_link_libraries( shapelizer
	${catkin_LIBRARIES}
	${CGAL_LIBRARIES}
)

add_executable(filter
	src/filter.cpp
)
target_link_libraries( filter
	${catkin_LIBRARIES}
	${CGAL_LIBRARIES}
	${PCL_LIBRARIES}
)


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
# install(TARGETS shape_it shape_it_node
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

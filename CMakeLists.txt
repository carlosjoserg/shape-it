cmake_minimum_required(VERSION 2.8.3)
project(visual_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs 
  geometry_msgs shape_msgs sensor_msgs visualization_msgs urdf tf pcl_conversions pcl_ros pcl_msgs)



## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL 1.7 REQUIRED)

link_directories(
  ${PCL_LIBRARY_DIRS}
  )
add_definitions(
  ${PCL_DEFINITIONS}
  )


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
add_message_files(
  FILES
  Cylinder.msg
  Table.msg
  TabletopDetectionResult.msg
  Sphere.msg
  Cone.msg
  Parallelepiped.msg
  Plane.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
add_service_files(
  FILES
  SegmentObjectInHand.srv
  TabletopSegmentation.srv
  ConeFitting.srv
  SphereFitting.srv
  CylinderFitting.srv
  ObjectManaging.srv
  ParallelepipedFitting.srv
  ObjectManaging.srv
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  shape_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS roscpp
   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(marker_generator src/marker_generator.cpp)

## Declare a cpp executable
add_executable(tabletop_segmentation src/tabletop_segmentation.cpp)
add_executable(cylinder_fitting src/cylinder_fitting.cpp)
#add_executable(ping_cylinder_fitting src/ping_cylinder_fitting.cpp)
add_executable(ping_tabletop_node src/ping_tabletop_node.cpp)
add_executable(object_managing src/object_managing.cpp)
add_executable(ping_ob_mg src/ping_ob_mg.cpp)
add_executable(sphere_fitting src/sphere_fitting.cpp)
add_executable(cone_fitting src/cone_fitting.cpp)
add_executable(parallelepiped_fitting src/parallelepiped_fitting.cpp)







## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(tabletop_segmentation tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)
add_dependencies(cylinder_fitting tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)
#add_dependencies(ping_cylinder_fitting tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)
add_dependencies(ping_tabletop_node tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)
add_dependencies(object_managing tabletop_segmentation_gencpp visual_perception_genlisp marker_generator  visual_perception_gen_cpp grasp_planner_gen_cpp)
add_dependencies(ping_ob_mg tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)
add_dependencies(sphere_fitting tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)
add_dependencies(cone_fitting tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)
add_dependencies(parallelepiped_fitting tabletop_segmentation_gencpp visual_perception_genlisp marker_generator)


## Specify libraries to link a library or executable target against
target_link_libraries(tabletop_segmentation marker_generator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cylinder_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(ping_cylinder_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ping_tabletop_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(object_managing ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ping_ob_mg ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(sphere_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cone_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(parallelepiped_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})





#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS phobic_robot_demo phobic_robot_demo_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phobic_robot_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


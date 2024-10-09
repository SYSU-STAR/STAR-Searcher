cmake_minimum_required(VERSION 2.8.3)
project(fuae_gazebo)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mav_msgs
  nav_msgs
  sensor_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs 
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR} 
)

# add_executable(fuae_gazebo_node
#     src/bounce_test_example.cpp 
#     )
# target_link_libraries(fuae_gazebo_node 
#     ${catkin_LIBRARIES})

# add_executable(position_controller_node
#     src/position_controller_node.cpp
#     )
# target_link_libraries(position_controller_node 
#     ${catkin_LIBRARIES})

# add_executable(poly_controller_node
#     src/poly_controller_node.cpp
# )
# target_link_libraries(poly_controller_node
#     ${catkin_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

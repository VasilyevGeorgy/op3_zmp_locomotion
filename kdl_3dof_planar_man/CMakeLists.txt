cmake_minimum_required(VERSION 2.8.3)
project(kdl_3dof_planar_man)


find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    geometry_msgs
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    message_generation
    message_runtime
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

 add_service_files(
   FILES
   SetInitServer.srv
   IKServer.srv
   FKServer.srv
   FK_empty.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

add_executable(kdl_3dof_planar_man_services src/main.cpp src/planar_man_services.cpp src/3dof_planar_man.cpp)
add_executable(IK_server src/ik_server.cpp src/3dof_planar_man.cpp)

target_link_libraries(IK_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(kdl_3dof_planar_man_services ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${orocos_kdl_LIBRARIES})

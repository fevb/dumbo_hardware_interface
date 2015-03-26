cmake_minimum_required(VERSION 2.8.3)
project(dumbo_powercube_chain)

find_package(catkin REQUIRED COMPONENTS roscpp cob_srvs sensor_msgs control_msgs urdf diagnostic_updater kvaser_canlib brics_actuator diagnostic_msgs dumbo_srvs schunk_sdh)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)


catkin_package(
  DEPENDS 
  CATKIN_DEPENDS cob_srvs sensor_msgs control_msgs urdf diagnostic_updater kvaser_canlib brics_actuator diagnostic_msgs dumbo_srvs schunk_sdh
  INCLUDE_DIRS common/include
  LIBRARIES powercube_commands powercube_ctrl
)

include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

set(SDHLib ${PROJECT_SOURCE_DIR}/../../schunk_sdh/common/lib/kvaser_x86_64/libSDHLibrary-CPP.a)


add_library(powercube_commands common/src/powercube_commands.c)
target_link_libraries(powercube_commands ${catkin_LIBRARIES})
set_target_properties(powercube_commands PROPERTIES COMPILE_FLAGS "-D__LINUX__")


add_executable(${PROJECT_NAME} common/src/PowerCubeCtrl.cpp common/src/TimeStamp.cpp common/src/moveCommand.cpp common/src/powercube_commands_wrapper.cpp common/src/collision.c common/src/dumbo_kinematics.c common/src/homogenic_vectors.c common/src/PG70Gripper.cpp ros/src/pg70_node.cpp ros/src/sdh_node.cpp ros/src/dumbo_powercube_chain.cpp)
target_link_libraries(${PROJECT_NAME} rt powercube_commands ${SDHLib} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DWITH_ESD_CAN -DOSNAME_LINUX")
add_dependencies(${PROJECT_NAME} cob_srvs_gencpp brics_actuator_gencpp schunk_sdh_gencpp dumbo_srvs_gencpp)

add_executable(test_movevel test/test_movevel.cpp)
target_link_libraries(test_movevel ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_movevel cob_srvs_gencpp brics_actuator_gencpp schunk_sdh_gencpp dumbo_srvs_gencpp)

add_executable(test_hand test/test_hand.cpp)
target_link_libraries(test_hand rt ${SDHLib} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
set_target_properties(test_hand PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DWITH_ESD_CAN -DOSNAME_LINUX")
add_dependencies(test_hand cob_srvs_gencpp brics_actuator_gencpp schunk_sdh_gencpp dumbo_srvs_gencpp)

add_library(powercube_ctrl common/src/PG70Gripper.cpp common/src/PowerCubeCtrl.cpp common/src/powercube_commands_wrapper.cpp common/src/collision.c common/src/dumbo_kinematics.c common/src/homogenic_vectors.c)
target_link_libraries(powercube_ctrl powercube_commands ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME} test_hand test_movevel powercube_ctrl
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



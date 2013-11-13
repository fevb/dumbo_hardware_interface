cmake_minimum_required(VERSION 2.8.3)
project(dumbo_force_torque_sensor)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs kvaser_canlib cob_srvs dumbo_powercube_chain tf sensor_msgs)

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS geometry_msgs kvaser_canlib cob_srvs dumbo_powercube_chain tf sensor_msgs
  INCLUDE_DIRS common/include
  LIBRARIES 
)

include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(dumbo_force_torque common/src/ForceTorqueSensor.cpp common/src/ComunicationCanx.cpp common/src/ft_sensor_function.cpp ros/src/dumbo_force_torque.cpp)
target_link_libraries(dumbo_force_torque rt canlib ${catkin_LIBRARIES})
set_target_properties(dumbo_force_torque PROPERTIES COMPILE_FLAGS "-D__LINUX__")

add_executable(getserialnumber test/getserialnumber.cpp common/src/ForceTorqueSensor.cpp common/src/ComunicationCanx.cpp common/src/ft_sensor_function.cpp)
target_link_libraries(getserialnumber rt canlib ${catkin_LIBRARIES})

install(TARGETS dumbo_force_torque getserialnumber
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
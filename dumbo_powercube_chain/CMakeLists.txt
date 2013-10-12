cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add project executable
#rosbuild_add_executable(dumbo_powercube_chain_col common/src/PowerCubeCtrl.cpp common/src/TimeStamp.cpp common/src/moveCommand.cpp common/src/powercube_commands_wrapper_col.cpp common/src/powercube_commands.c common/src/collision.c common/src/dumbo_kinematics.c common/src/homogenic_vectors.c ros/src/dumbo_powercube_chain.cpp)
rosbuild_add_library(powercube_commands common/src/powercube_commands.c)

rosbuild_add_executable(dumbo_powercube_chain common/src/PowerCubeCtrl.cpp common/src/TimeStamp.cpp common/src/moveCommand.cpp common/src/powercube_commands_wrapper.cpp common/src/powercube_commands.c common/src/collision.c common/src/dumbo_kinematics.c common/src/homogenic_vectors.c common/src/PG70Gripper.cpp ros/src/pg70_node.cpp ros/src/sdh_node.cpp ros/src/dumbo_powercube_chain.cpp)

rosbuild_add_executable(test_movevel test/test_movevel.cpp)
rosbuild_add_executable(test_hand test/test_hand.cpp)

#rosbuild_add_compile_flags(dumbo_powercube_chain_col -D__LINUX__)
#target_link_libraries(dumbo_powercube_chain_col rt)


rosbuild_add_compile_flags(dumbo_powercube_chain -D__LINUX__ -DWITH_ESD_CAN -DOSNAME_LINUX)
rosbuild_add_compile_flags(test_hand -D__LINUX__ -DWITH_ESD_CAN -DOSNAME_LINUX)
rosbuild_link_boost(dumbo_powercube_chain thread)


target_link_libraries(dumbo_powercube_chain rt SDHLibrary-CPP canserial_kvaser)
target_link_libraries(test_hand rt SDHLibrary-CPP canserial_kvaser)

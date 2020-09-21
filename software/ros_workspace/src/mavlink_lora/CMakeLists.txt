cmake_minimum_required(VERSION 3.7)
project(mavlink_lora)

# compile as c++11. Supported in kinetic and forward
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    mavlink_msgs
    )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    mavlink_msgs
)

##############################################################################
# Sources
##############################################################################
file(GLOB_RECURSE HPP_INCLUDES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)
file(GLOB_RECURSE CPP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE H_INCLUDES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB_RECURSE C_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.c)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

##############################################################################
# Binaries
##############################################################################
add_executable(${PROJECT_NAME}_node ${CPP_SOURCES} ${HPP_INCLUDES} ${C_SOURCES} ${H_INCLUDES})

# needed because the message file needs to be maked before the node using it
#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )

##############################################################################
# Test -  Add gtest based cpp test target and link libraries
##############################################################################
if (CATKIN_ENABLE_TESTING)

    # ROStest for unit and integration test
    find_package(rostest REQUIRED)

    # Add gtest based cpp test target and link libraries. This function actually makes a test-executable
    # REMEMBER to add lib files too to not get undefined reference
    # alternative we can compile as lib and link the lib instead.
    #catkin_add_gtest(${PROJECT_NAME}-test
    #        src/mavlink_lora_lib.cpp
    #        src/serial.c)

    # link target as usual
    #target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif () # CATKIN_ENABLE_TESTING

cmake_minimum_required(VERSION 2.8.3)
project(elektron_ids)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf

        nav_msgs
        geometry_msgs
        sensor_msgs
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            tf
        
            nav_msgs
            geometry_msgs
            sensor_msgs
        )

add_library(${PROJECT_NAME}
        src/ComponentIDS.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_executable(${PROJECT_NAME}-main
        src/main.cpp
        )

target_link_libraries(${PROJECT_NAME}-main
        ${PROJECT_NAME}
        )

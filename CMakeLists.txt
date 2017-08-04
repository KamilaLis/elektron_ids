project(elektron_ids)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

##############
## Packages ##
##############

## Catkin ##

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES elektron_ids
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Locations ##

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

## Libraries ##

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/ComponentIDS.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Executables ##

add_executable(elektron_ids_node
  src/elektron_ids_node.cpp
)

target_link_libraries(elektron_ids_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


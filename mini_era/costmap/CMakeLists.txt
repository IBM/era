cmake_minimum_required(VERSION 2.8)
project(costmap)

find_package(OCTOMAP REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})


find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

set(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include)

add_library(costmap SHARED src/costmap.cpp)

target_include_directories(costmap PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${OCTOMAP_INCLUDE_DIRS}
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(costmap ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
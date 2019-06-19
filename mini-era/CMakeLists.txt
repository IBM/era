cmake_minimum_required (VERSION 2.8.11)
project (mini-era)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(object_detection)
add_subdirectory(costmap)


set(CMAKE_BUILD_TYPE RelWithDebInfo)

SET(CMAKE_CXX_FLAGS_DEBUG "-O0")
SET(CMAKE_C_FLAGS_DEBUG "-O0")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -O0 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

include(ExternalProject)
ExternalProject_Add(
    model
    PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/models"
    URL http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_2018_01_28.tar.gz
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)

include_directories(include)

set(MINI_ERA_SOURCE_FILES
    src/mini_era.cpp)

add_executable(mini_era ${MINI_ERA_SOURCE_FILES})
target_link_libraries(mini_era ${OpenCV_LIBS} object_detection costmap)

add_dependencies(mini_era model)

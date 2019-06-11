cmake_minimum_required (VERSION 2.8.3)
project (object_detection)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE RelWithDebInfo)

SET(CMAKE_CXX_FLAGS_DEBUG "-O0")
SET(CMAKE_C_FLAGS_DEBUG "-O0")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -O0 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )


#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

set(TENSORFLOW_INCLUDE_DIRS 
/usr/local/include/tf/ 
/usr/local/include/tf/bazel-genfiles/ 
/usr/local/include/tf/tensorflow/ 
/usr/local/include/tf/third-party/
/usr/include/eigen3
/home/nuc/.local/lib/python2.7/site-packages/tensorflow/include
)

include_directories(include)
include_directories("/usr/local/include/tf/")
include_directories("/usr/local/include/tf/bazel-genfiles/")
include_directories("/usr/local/include/tf/tensorflow/")
include_directories("/usr/local/include/tf/third-party/")
include_directories("/usr/include/eigen3")
include_directories("/home/nuc/.local/lib/python2.7/site-packages/tensorflow/include")
include_directories("/usr/include")


add_library(object_detection SHARED src/object_detection.cpp src/utils.cpp)

target_include_directories(object_detection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${TENSORFLOW_INCLUDE_DIRS}
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


target_link_libraries(object_detection "/usr/local/lib/libtensorflow_cc.so")
target_link_libraries(object_detection "/usr/local/lib/libtensorflow_framework.so")

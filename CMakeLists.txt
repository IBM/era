cmake_minimum_required(VERSION 2.8.11)
project(ERA)

set(CMAKE_BUILD_TYPE Debug)

include_directories(include)

add_library(occgrid STATIC src/occgrid.c)

add_executable(era src/main.c)

target_link_libraries(era LINK_PUBLIC occgrid m)
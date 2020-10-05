cmake_minimum_required(VERSION 2.8.11)
project(ERA)

#variable_watch(CMAKE_BUILD_TYPE)
#set (CMAKE_BUILD_TYPE "Debug" CACHE STRING "Default build type: Debug" FORCE)
#set (CMAKE_BUILD_TYPE "Release" CACHE STRING "Default build type: Release" FORCE)

# None of this seems to work at all...
#set(CMAKE_BUILD_TYPE Debug)
#set(default_build_type "Debug")
#if (NOT CMAKE_BUILD_TYPE)
#  set (CMAKE_BUILD_TYPE "Debug" CACHE STRING "Default build type: Debug" FORCE)
#endif()

include_directories(include)

add_library(occgrid STATIC src/occgrid.c)
add_library(lz4 STATIC src/lz4.c)
add_library(sdr STATIC src/complex_ops.c src/crc.c src/delay.c src/descrambler_function.c src/fft.c src/fir.c src/gr_equalizer.c src/ofdm.c src/recv_pipe.c src/sdr_descrabler.c src/sdr_viterbi.c src/simple_dft.c src/sync_long.c src/sync_short.c src/viterbi_flat.c src/xmit_pipe.c)


add_executable(era1 src/main.c)
add_executable(era2 src/main.c)

target_compile_definitions(era1 PRIVATE BAG_PORT=5556)
target_compile_definitions(era2 PRIVATE BAG_PORT=5557)

target_compile_definitions(era1 PRIVATE XMIT_PORT=5558)
target_compile_definitions(era2 PRIVATE XMIT_PORT=5560)

target_compile_definitions(era1 PRIVATE RECV_PORT=5559)
target_compile_definitions(era2 PRIVATE RECV_PORT=5561)

target_compile_definitions(era1 PRIVATE IMAGE_FN="gridimage_era1_")
target_compile_definitions(era2 PRIVATE IMAGE_FN="gridimage_era2_")

target_link_libraries(era1 LINK_PUBLIC occgrid lz4 sdr m)
target_link_libraries(era2 LINK_PUBLIC occgrid lz4 sdr m)

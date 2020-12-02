cmake_minimum_required(VERSION 2.8.11)
project(ERA)

# define the C cross compiler to use
set(CMAKE_C_COMPILER /opt/riscv/bin/riscv64-unknown-linux-gnu-gcc)

# define the ++C cross compiler to use
set(CMAKE_CXX_COMPILER /opt/riscv/bin/riscv64-unknown-linux-gnu-g++)

# define the root location of hte RISC-V libraries and includes
set(CMAKE_FIND_ROOT_PATH /opt/riscv)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# define and force use of C99 mode.
macro(use_c99)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_C_FLAGS "-std=c99 ${CMAKE_C_FLAGS}")
    endif ()
  else ()
    set (CMAKE_STANDARD_99) 
  endif ()
endmacro(use_c99)

use_c99()

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

#add_library(occgrid STATIC src/occgrid.c)
#add_library(lz4 STATIC src/lz4.c)
#add_library(sdr STATIC src/complex_ops.c src/crc.c src/delay.c src/fft.c src/fir.c src/gr_equalizer.c src/ofdm.c src/recv_pipe.c src/sdr_descrambler.c src/sdr_viterbi.c src/simple_dft.c src/sync_long.c src/sync_short.c src/xmit_pipe.c)
#add_executable(era1 src/main.c)
#add_executable(era2 src/main.c)


add_executable(era1 src/main.c src/occgrid.c src/lz4.c src/complex_ops.c src/crc.c src/delay.c src/fft.c src/fir.c src/gr_equalizer.c src/ofdm.c src/recv_pipe.c src/sdr_descrambler.c src/sdr_viterbi.c src/simple_dft.c src/sync_long.c src/sync_short.c src/xmit_pipe.c)
add_executable(era2 src/main.c src/occgrid.c src/lz4.c src/complex_ops.c src/crc.c src/delay.c src/fft.c src/fir.c src/gr_equalizer.c src/ofdm.c src/recv_pipe.c src/sdr_descrambler.c src/sdr_viterbi.c src/simple_dft.c src/sync_long.c src/sync_short.c src/xmit_pipe.c)

add_executable(do_recv_pipe src/recv_main.c src/complex_ops.c src/delay.c src/fft.c src/fir.c src/gr_equalizer.c src/ofdm.c src/recv_pipe.c src/sdr_descrambler.c src/sdr_viterbi.c src/sync_long.c src/sync_short.c)
add_executable(do_xmit_pipe src/xmit_main.c src/crc.c src/fft.c src/xmit_pipe.c)

target_compile_definitions(era1 PRIVATE USE_ESP_INTERFACE)
target_compile_definitions(era2 PRIVATE USE_ESP_INTERFACE)
target_compile_definitions(do_recv_pipe PRIVATE USE_ESP_INTERFACE)
target_compile_definitions(do_xmit_pipe PRIVATE USE_ESP_INTERFACE)

target_compile_definitions(era1 PRIVATE INT_TIME)
target_compile_definitions(era2 PRIVATE INT_TIME)
target_compile_definitions(do_recv_pipe PRIVATE INT_TIME)
target_compile_definitions(do_xmit_pipe PRIVATE INT_TIME)

target_compile_definitions(era1 PRIVATE BAG_PORT=5556)
target_compile_definitions(era2 PRIVATE BAG_PORT=5557)

target_compile_definitions(era1 PRIVATE XMIT_PORT=5558)
target_compile_definitions(era2 PRIVATE XMIT_PORT=5560)

target_compile_definitions(era1 PRIVATE RECV_PORT=5559)
target_compile_definitions(era2 PRIVATE RECV_PORT=5561)

target_compile_definitions(era1 PRIVATE IMAGE_FN="gridimage_era1_")
target_compile_definitions(era2 PRIVATE IMAGE_FN="gridimage_era2_")

#target_link_libraries(era1 LINK_PUBLIC occgrid lz4 sdr m)
#target_link_libraries(era2 LINK_PUBLIC occgrid lz4 sdr m)

target_link_libraries(era1 LINK_PUBLIC m)
target_link_libraries(era2 LINK_PUBLIC m)
target_link_libraries(do_recv_pipe LINK_PUBLIC m)
target_link_libraries(do_xmit_pipe LINK_PUBLIC m)

# This is for the cross-compile to ESP-FPGA with Hardware Accelerators
include_directories(../../../esp/soft/ariane/drivers/include)

target_compile_definitions(era1 PRIVATE COMPILE_TO_ESP)
target_compile_definitions(era2 PRIVATE COMPILE_TO_ESP)
target_compile_definitions(do_recv_pipe PRIVATE COMPILE_TO_ESP)
target_compile_definitions(do_xmit_pipe PRIVATE COMPILE_TO_ESP)

target_compile_definitions(era1 PRIVATE HW_VIT)
target_compile_definitions(era2 PRIVATE HW_VIT)
target_compile_definitions(do_recv_pipe PRIVATE HW_VIT)
target_compile_definitions(do_xmit_pipe PRIVATE HW_VIT)

link_directories(../../../esp/soft/ariane/drivers/contig_alloc)
link_directories(../../../esp/soft/ariane/drivers/test)
link_directories(../../../esp/soft/ariane/drivers/libesp)
target_link_libraries(era1 LINK_PUBLIC rt)
target_link_libraries(era1 LINK_PUBLIC esp)
target_link_libraries(era1 LINK_PUBLIC test contig)
target_link_libraries(era1 LINK_PUBLIC contig)
target_link_libraries(era2 LINK_PUBLIC rt)
target_link_libraries(era2 LINK_PUBLIC esp)
target_link_libraries(era2 LINK_PUBLIC test contig)
target_link_libraries(era2 LINK_PUBLIC contig)
target_link_libraries(do_recv_pipe LINK_PUBLIC rt)
target_link_libraries(do_recv_pipe LINK_PUBLIC esp)
target_link_libraries(do_recv_pipe LINK_PUBLIC test)
target_link_libraries(do_recv_pipe LINK_PUBLIC contig)
target_link_libraries(do_xmit_pipe LINK_PUBLIC rt)
target_link_libraries(do_xmit_pipe LINK_PUBLIC esp)
target_link_libraries(do_xmit_pipe LINK_PUBLIC test)
target_link_libraries(do_xmit_pipe LINK_PUBLIC contig)

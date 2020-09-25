cmake_minimum_required(VERSION 2.8.11)
project(ERA)

set(CMAKE_BUILD_TYPE Debug)

include_directories(include)

add_library(occgrid STATIC src/occgrid.c)
add_library(lz4 STATIC src/lz4.c)
add_library(sdr STATIC src/complex_ops.c src/crc.c src/delay.c src/descrambler_function.c src/fft.c src/fir.c src/gr_equalizer.c src/ofdm.c src/recv_pipe.c src/sdr_descrabler.c src/sdr_viterbi.c src/simple_dft.c src/sync_long.c src/sync_short.c src/viterbi_flat.c src/xmit_pipe.c)


add_executable(era src/main.c)

target_link_libraries(era LINK_PUBLIC occgrid lz4 sdr m)

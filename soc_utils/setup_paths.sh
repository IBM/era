#!/bin/bash
####### THESE VARIABLES NEED TO BE SET! #########
export FPGA_HOST='192.168.1.99'
export FPGA_HOST_IP=9.2.212.205
export FPGA_USERNAME=aporva

export HPVM_DIR=/home/espuser/hpvm-release/hpvm
export RISCV_BIN_DIR=/home/espuser/riscv/bin
export ESP_ROOT=/home/espuser/esp
export CONDA_ENV_PATH=/home/espuser/.local/conda/envs/era38
export SOC_LIB_DIR=/home/espuser/scheduler-library-hpvm/
export SCHED_CONFIG=soc_utils/config_files/base_me_p3.config 


####### THESE VARIABLES SHOULD NOT NEED ANY MODIFICATION! #########
SH="$(readlink -f /proc/$$/exe)"
if [[ "$SH" == "/bin/zsh" ]]; then
  CUR_DIR="${0:A:h}"
else
  CUR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
fi
export HPVM_BUILD_DIR=$HPVM_DIR/build
export LD_LIBRARY_PATH=$HPVM_DIR/build/lib:$LD_LIBRARY_PATH

export PATH=$PATH:/$RISCV_BIN_DIR
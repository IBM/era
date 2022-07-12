#!/bin/bash
####### THESE VARIABLES NEED TO BE SET! #########
#export HPVM_DIR=$HOME/work_dir/hpvm-dssoc/hpvm
#export RISCV_BIN_DIR=$HOME/work_dir/riscv/bin
export HPVM_DIR=$HOME/hpvm_dev_epochs/hpvm/hpvm
export RISCV_BIN_DIR=$HOME/esp/esp/riscv/bin
export ESP_ROOT=$HOME/esp/esp


####### THESE VARIABLES SHOULD NOT NEED ANY MODIFICATION! #########
SH="$(readlink -f /proc/$$/exe)"
if [[ "$SH" == "/bin/zsh" ]]; then
  CUR_DIR="${0:A:h}"
else
  CUR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
fi
export SOC_LIB_DIR=$HOME/scheduler_cpy/scheduler/scheduler-library
export HPVM_BUILD_DIR=$HPVM_DIR/build
export LD_LIBRARY_PATH=$HPVM_DIR/build/lib:$LD_LIBRARY_PATH

export PATH=$PATH:/$RISCV_BIN_DIR

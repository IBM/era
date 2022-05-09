#!/bin/bash
####### THESE VARIABLES NEED TO BE SET! #########
#export HPVM_DIR=$HOME/work_dir/hpvm-dssoc/hpvm
#export RISCV_BIN_DIR=$HOME/work_dir/riscv/bin
export HPVM_DIR=/dccstor/epochs/aporvaa/hpvm/hpvm
export RISCV_BIN_DIR=/dccstor/epochs/aporvaa/riscv/bin
export ESP_ROOT=/dccstor/epochs/aporvaa/esp


####### THESE VARIABLES SHOULD NOT NEED ANY MODIFICATION! #########
SH="$(readlink -f /proc/$$/exe)"
if [[ "$SH" == "/bin/zsh" ]]; then
  CUR_DIR="${0:A:h}"
else
  CUR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
fi
export SOC_LIB_DIR=/dccstor/epochs/aporvaa/sched/scheduler-library-hpvm/
export HPVM_BUILD_DIR=$HPVM_DIR/build_debug2
export LD_LIBRARY_PATH=$HPVM_DIR/build/lib:$LD_LIBRARY_PATH

export PATH=$PATH:/$RISCV_BIN_DIR
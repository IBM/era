#!/bin/bash
####### THESE VARIABLES NEED TO BE SET! #########
#export HPVM_DIR=$HOME/work_dir/hpvm-dssoc/hpvm
#export RISCV_BIN_DIR=$HOME/work_dir/riscv/bin
export HPVM_DIR=/dccstor/epochs/aporvaa/hpvm/hpvm
export RISCV_BIN_DIR=/dccstor/epochs/aporvaa/riscv/bin
export ESP_ROOT=/dccstor/epochs/aporvaa/esp
export CONDA_ENV_PATH=/dccstor/epochs/aporvaa/.local/conda/envs/era38
export SOC_LIB_DIR=/dccstor/epochs/aporvaa/sched/scheduler-library-x86/
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
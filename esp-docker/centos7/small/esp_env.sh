#!/bin/bash

#
# Set ESP environment variables (no CAD tools)
#

# xilinx vivado environment
export XILINX_VIVADO=/

# risc-v toolchain environment
export RISCV=/home/espuser/riscv
export RISCV32IMC=/home/espuser/riscv32imc
export PATH=$PATH:/home/espuser/riscv/bin:/home/espuser/riscv32imc/bin
echo "Exported RISC-V toolchain environment"

# leon3 toolchain environment
export PATH=$PATH:/home/espuser/leon/bin
export PATH=$PATH:/home/espuser/leon/mklinuximg
export PATH=$PATH:/home/espuser/leon/sparc-elf/bin
echo "Exported Leon3 toolchain environment"

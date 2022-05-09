# Set this variable to the path containing the build folder for HPVM
HPVM_SRC=$HOME/hpvm/hpvm/hpvm

# REPLICATE THIS FOR ALL .C FILES IN MAKE; LINK THEM LATER

# File you want to compile
CPP_FILE=../src/main.c

OPT=$HPVM_SRC/build/bin/opt
CLANG=$HPVM_SRC/build/bin/clang
LLVM_LINK=$HPVM_SRC/build/bin/llvm-link
HCC=$HPVM_SRC/build/bin/hpvm-extract-task
HPVM_DEF_FILE=HPVMCFunctionDeclarations.ll
LIB=$HPVM_SRC/build/lib

# Compile file to llvmn bitcode format
$CLANG $CPP_FILE -O1 -S -emit-llvm -o era.ll -I../include -I./


# Run HeteroC++ frontend (If the input is already in HPVM-C format, like-in mini-era skip this command)
$HCC -declsfile $HPVM_DEF_FILE -o era.bc era.ll

#FOCUS ON ABOVE THIS POINT


# HPVM Backend specific passes
#$OPT -load $LIB/LLVMGenHPVM.so -genhpvm -globaldce -hpvm-timers-gen -S era.hetero.ll -o era.genhpvm.ll

# TRY RUNNING THE ONE ON TOP


# Set these paths appropriately
#SCHED_LIB_PATH=$HOME/scheduler/scheduler-library/sched_library/
#SCHED_CONFIG_PATH=$HOME/scheduler/scheduler-library/examples/mini-era/config_files/base_me_p2.config
#TASK_LIB_PATH=$HOME/scheduler/scheduler-library/task_library/
#TASK_CONFIG_PATH=$HOME/scheduler/scheduler-library/task_library/task_lib.config

# Scheduler backend invokation
#$OPT -load $LIB/LLVMBuildDFG.so -load $LIB/LLVMDFG2LLVM_EPOCHS.so -load $LIB/LLVMClearDFG.so -dfg2llvm-epochs -clearDFG -dce -globaldce -S era.genhpvm.ll -o era.host.ll -sched-lib-path=$SCHED_LIB_PATH -sched-config=$SCHED_CONFIG_PATH -task-config=$TASK_CONFIG_PATH -task-lib-path=$TASK_LIB_PATH

# LINK HPVM RT
#$LLVM_LINK $HPVM_SRC/build/tools/hpvm/projects/hpvm-rt/hpvm-rt.bc era.host.ll -S -o era.linked.ll

# Compile to object or executable
#$CLANG -O2 -lm -lrt -pthread -lOpenCL era.linked.ll -o era.hpvm.o


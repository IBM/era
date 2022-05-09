# Set this variable to the path containing the build folder for HPVM
HPVM_SRC=$(HOME)/hpvm/hpvm/hpvm

# REPLICATE THIS FOR ALL .C FILES IN MAKE; LINK THEM LATER

# File you want to compile
CPP_FILE=./src/main.c

OPT=$(HPVM_SRC)/build/bin/opt
CLANG=$(HPVM_SRC)/build/bin/clang
LLVM_LINK=$(HPVM_SRC)/build/bin/llvm-link
HCC=$(HPVM_SRC)/build/bin/hpvm-extract-task
HPVM_DEF_FILE=./era_hpvm_utils/HPVMCFunctionDeclarations.ll
LIB=$(HPVM_SRC)/build/lib

# Compile file to llvm bitcode format
main.o : $(CPP_FILE)
	$(CLANG) -O1 -S -emit-llvm -I./include -I./era_hpvm_utils $(CPP_FILE) -o era.ll
	$(HCC) -declsfile $(HPVM_DEF_FILE) -o era.bc era.ll
	hpvm-clang --hetero-cc --hpvm-target cpu -I./include $(CPP_FILE)  main.cpu



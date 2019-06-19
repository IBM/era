INPUT=fullhd
BMARK=sift
CC=gcc
override CFLAGS += -DGCC -DCHECK -D$(INPUT)

COMMON_DIR := ../common
C_DIR := ./src
C_SRC := $(wildcard $(C_DIR)/*.c)
COMMON_SRC := $(wildcard $(COMMON_DIR)/*.c)
DATA_DIR := ./data/$(INPUT)
C_RESULT := ../$(BMARK)/result
# RULES

EXE =
INCLUDES = -I$(COMMON_DIR) -I$(C_DIR)
COMPILE_C = $(CC) $(CFLAGS) -O2 $(INCLUDES)
COMPILE_MATH = -lm


compile: $(C_SRC)
	@echo 
	@$(COMPILE_C) $(COMMON_SRC) $(C_SRC) -o $(BMARK)$(EXE) -lm

c-run: compile
	@echo -e "Data set\t\t- $(INPUT)" 
	@./$(BMARK)$(EXE) $(DATA_DIR) $(C_RESULT) 


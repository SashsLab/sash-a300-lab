###############################################################################
#
#  	    File        : Makefile
#
#       Abstract    : Example Makefile for a C Project
#
#       Environment : Atollic TrueSTUDIO(R)
#
###############################################################################

SHELL=cmd

# System configuration
CC = arm-atollic-eabi-gcc
RM=rm -rf

# Assembler, Compiler and Linker flags and linker script settings
LINKER_FLAGS=-lm -mthumb -mcpu=cortex-m3  -Wl,--gc-sections -T$(LINK_SCRIPT) -static  -Wl,--start-group -lc_s -lm -Wl,--end-group -Wl,-cref "-Wl,-Map=$(BIN_DIR)/Uart_test.map" -Wl,--defsym=malloc_getpagesize_P=0x1000
LINK_SCRIPT="efm32_flash.ld"
ASSEMBLER_FLAGS=-c -g -O0 -mcpu=cortex-m3  -mthumb  -x assembler-with-cpp  -ILibraries\CMSIS\Include -ILibraries\CMSIS\RTOS -ILibraries\Device\EnergyMicro\EFM32\Include -ILibraries\emlib\inc -Iboard\bsp -Iboard\drivers -Iboard\config
COMPILER_FLAGS=-c -g -mcpu=cortex-m3  -O0 -Wall -ffunction-sections -fdata-sections -mthumb -D"EFM32GG990F1024"   -ILibraries\CMSIS\Include -ILibraries\CMSIS\RTOS -ILibraries\Device\EnergyMicro\EFM32\Include -ILibraries\emlib\inc -Iboard\bsp -Iboard\drivers -Iboard\config 

# Define output directory
OBJECT_DIR = Debug
BIN_DIR = $(OBJECT_DIR)

# Define sources and objects
SRC := $(wildcard */*/*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*.c) \
	$(wildcard */*/*/*/*.c) \
	$(wildcard */*/*/*.c) \
	$(wildcard */*/*.c) \
	$(wildcard */*.c)
SRCSASM := 	$(wildcard */*/*/*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*.s) \
	$(wildcard */*/*/*/*.s) \
	$(wildcard */*/*/*.s) \
	$(wildcard */*/*.s) \
	$(wildcard */*.s)
OBJS := $(SRC:%.c=$(OBJECT_DIR)/%.o) $(SRCSASM:%.s=$(OBJECT_DIR)/%.o)
OBJS := $(OBJS:%.S=$(OBJECT_DIR)/%.o)  

###############
# Build project
# Major targets
###############
all: buildelf

buildelf: $(OBJS) 
	$(CC) -o "$(BIN_DIR)/Uart_test.elf" $(OBJS) $(LINKER_FLAGS)

clean:
	$(RM) $(OBJS) "$(BIN_DIR)/Uart_test.elf" "$(BIN_DIR)/Uart_test.map"


##################
# Specific targets
##################
$(OBJECT_DIR)/src/main.o: src/main.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) src/main.c -o $(OBJECT_DIR)/src/main.o 


##################
# Implicit targets
##################
$(OBJECT_DIR)/%.o: %.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) $< -o $@

$(OBJECT_DIR)/%.o: %.s
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.S
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@

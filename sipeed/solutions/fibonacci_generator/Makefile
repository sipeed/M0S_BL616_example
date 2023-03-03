SIPEED_SOLUTION_PATH ?= .
BL_SDK_BASE ?= $(SIPEED_SOLUTION_PATH)/../../..

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl616dk
CROSS_COMPILE ?= riscv64-unknown-elf-

# add custom cmake definition
#cmake_definition+=-Dxxx=sss

include $(BL_SDK_BASE)/project.build

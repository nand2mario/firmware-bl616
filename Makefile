SDK_DEMO_PATH ?= .
BL_SDK_BASE ?= ../bouffalo_sdk_harbaum
TANG_BOARD ?= mega60k

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl616dk
CROSS_COMPILE ?= riscv64-unknown-elf-

cmake_definition+=-DTANG_BOARD=$(TANG_BOARD)

include $(BL_SDK_BASE)/project.build


#
#             LUFA Library
#     Copyright (C) Dean Camera, 2012.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.crg
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

MCU          := atmega32u4
#MCU          := atmega8
ARCH         := AVR8
BOARD        := PROMICRO
F_CPU        := 16000000
F_USB        := $(F_CPU)
OPTIMIZATION := s
TARGET       := grbl
SRC          = main.c motion_control.c gcode.c serial.c \
               protocol.c stepper.c report.c system.c spindle_control.c \
               eeprom.c settings.c planner.c nuts_bolts.c limits.c print.c config/Descriptors.c \
			 $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    := deps/LUFA
CC_FLAGS     = -DCPU_MAP_ATMEGA32U4 -DUSE_LUFA_CONFIG_HEADER -Iconfig/
LD_FLAGS     :=
FUSES        := -U hfuse:w:0xd2:m -U lfuse:w:0xff:m

COMPILE      = avr-gcc -Wall -Os -mmcu=$(MCU)
OUT_DIR      := obj
SOURCEDIR    := grbl
#PROGRAMMER   := -cavr109 -PCOM4 -b57600
#PROGRAMMER   := -cavrispmkII
PROGRAMMER   := -cstk500v2 -Pusb
#PROGRAMMER  := -cstk500v1 -PCOM9 -b19200 

AVRDUDE := C:\Users\mk\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17\bin\avrdude \
          -CC:\Users\mk\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17\etc\avrdude.conf \
          -v -p$(MCU) $(PROGRAMMER) -D

# if main.c:105:1: fatal error: error writing to -: Invalid argument
# need create obj folder 
#  mkdir obj

all: clean
	avr-size -C --mcu=$(MCU) $(TARGET).elf
	
# Include LUFA-specific DMBS extension modules
DMBS_LUFA_PATH ?= $(LUFA_PATH)/Build/LUFA
include $(DMBS_LUFA_PATH)/lufa-sources.mk
include $(DMBS_LUFA_PATH)/lufa-gcc.mk

# Include common DMBS build system modules
DMBS_PATH      ?= $(LUFA_PATH)/Build/DMBS/DMBS
include $(DMBS_PATH)/core.mk
include $(DMBS_PATH)/cppcheck.mk
include $(DMBS_PATH)/doxygen.mk
include $(DMBS_PATH)/dfu.mk
include $(DMBS_PATH)/gcc.mk
include $(DMBS_PATH)/hid.mk
include $(DMBS_PATH)/avrdude.mk
include $(DMBS_PATH)/atprogram.mk

flash:
	$(AVRDUDE) -e -U flash:w:grbl.hex:i
	
#FUSES
#Atmega 8	
#flash:
#	$(AVRDUDE) -F -e -Ulock:w:0x3F:m -Uefuse:w:0xcb:m -Uhfuse:w:0xd9:m -Ulfuse:w:0xff:m
#ATmega32u4
fuse:
	$(AVRDUDE) -e -Ulock:w:0x3F:m -Uefuse:w:0xcb:m -Uhfuse:w:0xd8:m -Ulfuse:w:0xff:m
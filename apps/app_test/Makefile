# Copyright (C) 2015 Raphaël Poggi. All rights reserved.
# Author: Raphaël Poggi <poggi.raph@gmail.com>
#
# Provided under the three clause BSD license found in the LICENSE file.

TARGET_NAME := app_test.elf

OBJS := app_test.o

MCPU=cortex-m4

INCLUDES	+= -I$(KERNEL_BASE)/include
INCLUDES	+= -I$(KERNEL_BASE)/boards
INCLUDES	+= -include $(KERNEL_BASE)/config.h
ASFLAGS	:= -g $(INCLUDES) -D__ASSEMBLY__ -mcpu=$(MCPU) -mthumb
CFLAGS  :=  -Wall -mlong-calls -fno-builtin -ffunction-sections 
CFLAGS += -mcpu=$(MCPU) -mthumb -nostdlib -g $(INCLUDES)
CFLAGS +=  -mword-relocations
#CFLAGS += -mcpu=$(MCPU) -mthumb -nostdlib -funwind-tables -g $(INCLUDES)
LDFLAGS	:= -g $(INCLUDES) -nostartfiles #-Wl,--gc-sections
LDFLAGS += -r

LD_SCRIPT = apps.ld

CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)as
AR := $(CROSS_COMPILE)ar
LD := $(CROSS_COMPILE)ld

all: $(TARGET_NAME)

clean:
	$(RM) $(TARGET_NAME)
	$(RM) $(OBJS)
	
$(TARGET_NAME): $(OBJS) 
	@@echo "LD " $@
	@$(CC) $(LDFLAGS) $(OBJS) -T$(LD_SCRIPT) -o $@

%.o: %.c
	@@echo "CC " $<
	@$(CC) $(CFLAGS) -c $< -o $@

#SOURCES_ASM := $(wildcard *.S)
#SOURCES_C   := $(wildcard *.c)
 
#OBJS        := $(patsubst %.S,%.o,$(SOURCES_ASM))
#OBJS        += $(patsubst %.c,%.o,$(SOURCES_C))

MACH=at91
SOC=sam7s

SAM7S_SRAM_LD=sram_sam7s.lds
SAM7S_FLASH_LD=flash_sam7s.lds
BCM2835_LD=link-arm-eabi.ld
SAM3X_LD=sam3x.ld
SAM3X8_SRAM_LD=sram.ld
LD_SCRIPT=$(SAM7S_SRAM_LD)
#LD_SCRIPT=$(SAM7S_FLASH_LD)

INCLUDES	+= -I$(KERNEL_BASE)/include
INCLUDES	+= -I$(KERNEL_BASE)/boards
ASFLAGS	:= -g $(INCLUDES) -D__ASSEMBLY__
CFLAGS  :=  -Wall -mlong-calls -fno-builtin -ffunction-sections -mcpu=arm7tdmi -nostdlib -g $(INCLUDES)
#CFLAGS  :=  -Wall -mlong-calls -fpic -ffunction-sections -mcpu=arm7tdmi -nostdlib -g $(INCLUDES)
#CFLAGS  :=  -Wall -mlong-calls -fpic -ffreestanding -nostdlib -g $(INCLUDES)
LDFLAGS	:= -g $(INCLUDES) -nostartfiles #-Wl,--gc-sections

OBJS	:= 	asm/head.o \
		arch/arm/kernel/svc_asm.o \
		arch/arm/kernel/svc.o \
		arch/arm/kernel/context.o \
		boards/mach-$(MACH)/$(SOC).o \
		boards/mach-$(MACH)/uart-$(SOC).o \
		boards/mach-$(MACH)/aic.o \
		boards/mach-$(MACH)/pit.o \
		boards/mach-$(MACH)/pio.o \
		boot/boot-$(SOC).o \
		drivers/uart-core.o \
		drivers/pit-core.o \
		drivers/pio-core.o \
		kernel/main.o \
		kernel/mutex.o \
		kernel/interrupt.o \
		kernel/scheduler.o \
		kernel/task.o \
		mm/alloc.o \
		mm/init.o \
		mm/free.o \
		utils/stdio.o \
		utils/utils.o

config:
	@@echo "CP mach-$(MACH)/board-$(SOC).h -> board.h"
	@cp boards/mach-$(MACH)/board-$(SOC).h boards/board.h
	@ln -s $(KERNEL_BASE)/arch/arm/include $(KERNEL_BASE)/include/arch

cscope:
	@cscope -b -q -k -R
 
all: kernel.img
 
include $(wildcard *.d)
 
kernel.elf: $(OBJS) 
	@@echo "LD " $@
	@$(CROSS_COMPILE)gcc $(LDFLAGS) $(OBJS) -T$(LD_SCRIPT) -o $@
 
kernel.img: kernel.elf
	@@echo "OBJCOPY " $<
	@$(CROSS_COMPILE)objcopy kernel.elf -O binary kernel.img
 
clean:
	$(RM) $(OBJS) kernel.elf kernel.img
	$(RM) boards/board.h
 
dist-clean: clean
	$(RM) `find . -name *.d`
 
%.o: %.c
	@@echo "CC " $<
	@$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@
 
%.o: %.S
	@@echo "CC " $<
	@$(CROSS_COMPILE)gcc $(ASFLAGS) -c $< -o $@

#SOURCES_ASM := $(wildcard *.S)
#SOURCES_C   := $(wildcard *.c)
 
#OBJS        := $(patsubst %.S,%.o,$(SOURCES_ASM))
#OBJS        += $(patsubst %.c,%.o,$(SOURCES_C))

ARMV=armv7m
MACH=stm32
SOC=stm32f407
MCPU=cortex-m4

SAM7S_SRAM_LD=sram_sam7s.lds
SAM7S_FLASH_LD=flash_sam7s.lds
BCM2835_LD=link-arm-eabi.ld
SAM3X_LD=sam3x.ld
SAM3X8_SRAM_LD=sram.ld
STM32F407_LD=stm32.ld
LD_SCRIPT=$(STM32F407_LD)
#LD_SCRIPT=$(SAM7S_SRAM_LD)
#LD_SCRIPT=$(SAM7S_FLASH_LD)

INCLUDES	+= -I$(KERNEL_BASE)/include
INCLUDES	+= -I$(KERNEL_BASE)/boards
ASFLAGS	:= -g $(INCLUDES) -D__ASSEMBLY__ -mcpu=$(MCPU) -mthumb
CFLAGS  :=  -Wall -mlong-calls -fno-builtin -ffunction-sections -mcpu=$(MCPU) -mthumb -nostdlib -g $(INCLUDES)
#CFLAGS  :=  -Wall -mlong-calls -fpic -ffunction-sections -mcpu=arm7tdmi -nostdlib -g $(INCLUDES)
#CFLAGS  :=  -Wall -mlong-calls -fpic -ffreestanding -nostdlib -g $(INCLUDES)
LDFLAGS	:= -g $(INCLUDES) -nostartfiles #-Wl,--gc-sections

OBJS	:= 	asm/head.o \
		arch/arm/$(ARMV)/kernel/svc_asm.o \
		arch/arm/$(ARMV)/kernel/svc.o \
		arch/arm/$(ARMV)/kernel/context.o \
		arch/arm/$(ARMV)/systick.o \
		arch/arm/$(ARMV)/handlers.o \
		boards/mach-$(MACH)/$(SOC).o \
		boot/boot-$(SOC).o \
		drivers/uart-core.o \
		kernel/main.o \
		kernel/mutex.o \
		kernel/scheduler.o \
		kernel/interrupt.o \
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
	@ln -s $(KERNEL_BASE)/arch/arm/$(ARMV)/include $(KERNEL_BASE)/include/$(ARMV)

cscope:
	@@echo "GEN " $@
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
	$(RM) include/arch
	$(RM) include/$(ARMV)
 
dist-clean: clean
	$(RM) `find . -name *.d`
 
%.o: %.c
	@@echo "CC " $<
	@$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@
 
%.o: %.S
	@@echo "CC " $<
	@$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

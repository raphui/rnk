#SOURCES_ASM := $(wildcard *.S)
#SOURCES_C   := $(wildcard *.c)
 
#OBJS        := $(patsubst %.S,%.o,$(SOURCES_ASM))
#OBJS        += $(patsubst %.c,%.o,$(SOURCES_C))

ARMV=armv7m
MACH=stm32
SOC=stm32f429
#SOC=stm32f407
MCPU=cortex-m4

SAM7S_SRAM_LD=sram_sam7s.lds
SAM7S_FLASH_LD=flash_sam7s.lds
BCM2835_LD=link-arm-eabi.ld
SAM3X_LD=sam3x.ld
SAM3X8_SRAM_LD=sram.ld
STM32F407_LD=stm32.ld
STM32F429_LD=stm32_alt.ld
LD_SCRIPT=$(STM32F429_LD)
#LD_SCRIPT=$(STM32F407_LD)
#LD_SCRIPT=$(SAM7S_SRAM_LD)
#LD_SCRIPT=$(SAM7S_FLASH_LD)
STM32_DEFINE = STM32_F429

KCONFIG_AUTOHEADER=config.h

INCLUDES	+= -I$(KERNEL_BASE)/include
INCLUDES	+= -I$(KERNEL_BASE)/boards
ASFLAGS	:= -g $(INCLUDES) -D__ASSEMBLY__ -mcpu=$(MCPU) -mthumb
CFLAGS  :=  -Wall -mlong-calls -fno-builtin -ffunction-sections -mcpu=$(MCPU) -mthumb -nostdlib -funwind-tables -g $(INCLUDES) -D$(STM32_DEFINE)
CFLAGS += -DUNWIND
#CFLAGS  :=  -Wall -mlong-calls -fpic -ffunction-sections -mcpu=arm7tdmi -nostdlib -g $(INCLUDES)
#CFLAGS  :=  -Wall -mlong-calls -fpic -ffreestanding -nostdlib -g $(INCLUDES)
LDFLAGS	:= -g $(INCLUDES) -nostartfiles #-Wl,--gc-sections

OBJS	:= 	asm/head.o \
		arch/arm/$(ARMV)/kernel/svc_asm.o \
		arch/arm/$(ARMV)/kernel/svc.o \
		arch/arm/$(ARMV)/kernel/context.o \
		arch/arm/$(ARMV)/nvic.o \
		arch/arm/$(ARMV)/systick.o \
		arch/arm/$(ARMV)/handlers.o \
		boards/mach-$(MACH)/$(SOC).o \
		boards/mach-$(MACH)/usart-$(MACH).o \
		boards/mach-$(MACH)/rcc-$(MACH).o \
		boards/mach-$(MACH)/pio-$(MACH).o \
		boards/mach-$(MACH)/timer-$(MACH).o \
		boards/mach-$(MACH)/i2c-$(MACH).o \
		boards/mach-$(MACH)/spi-$(MACH).o \
		boards/mach-$(MACH)/dma-$(MACH).o \
		boards/mach-$(MACH)/ltdc-$(MACH).o \
		boards/mach-$(MACH)/fmc-$(MACH).o \
		boards/mach-$(MACH)/exti-$(MACH).o \
		boot/boot-$(SOC).o \
		drivers/clk.o \
		drivers/pio.o \
		drivers/usart.o \
		drivers/timer.o \
		drivers/i2c.o \
		drivers/spi.o \
		drivers/dma.o \
		drivers/lcd.o \
		drivers/ili9341.o \
		kernel/main.o \
		kernel/mutex.o \
		kernel/scheduler.o \
		kernel/semaphore.o \
		kernel/interrupt.o \
		kernel/queue.o \
		kernel/task.o \
		kernel/time.o \
		mm/alloc.o \
		mm/init.o \
		mm/free.o \
		utils/backtrace.o \
		utils/stdio.o \
		utils/symbols.o \
		utils/string.o \
		utils/utils.o
conf:
	@@echo "CP mach-$(MACH)/board-$(SOC).h -> board.h"
	@cp boards/mach-$(MACH)/board-$(SOC).h boards/board.h
	@ln -s $(KERNEL_BASE)/boards/mach-$(MACH)/include $(KERNEL_BASE)/include/mach
	@ln -s $(KERNEL_BASE)/arch/arm/include $(KERNEL_BASE)/include/arch
	@ln -s $(KERNEL_BASE)/arch/arm/$(ARMV)/include $(KERNEL_BASE)/include/$(ARMV)

cscope:
	@@echo "GEN " $@
	@cscope -b -q -k -R
 
all: kernel.img 
 
include $(wildcard *.d)
 
symbols-make: 
	@@echo "GEN " $@
	@sh tools/sym.sh make

symbols-clean: 
	@@echo "CLEAN " $@
	@sh tools/sym.sh clean
 
kernel.elf: $(OBJS) 
	@@echo "LD " $@
	@$(CROSS_COMPILE)gcc $(LDFLAGS) $(OBJS) -T$(LD_SCRIPT) -o $@

kernel.img: kernel.elf 
	@@echo "OBJCOPY " $<
	@$(CROSS_COMPILE)objcopy kernel.elf -O binary kernel.bin
 
clean:	symbols-clean
	$(RM) $(OBJS) kernel.elf kernel.img
	$(RM) boards/board.h
	$(RM) include/arch
	$(RM) include/$(ARMV)
	$(RM) include/mach
 
dist-clean: clean
	$(RM) `find . -name *.d`
 
%.o: %.c config.h
	@@echo "CC " $<
	@$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@
 
%.o: %.S config.h
	@@echo "CC " $<
	@$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

.config:
	echo "ERROR: No config file loaded."
	exit 1

%_defconfig:
	cp arch/${ARCH}/configs/$@ .config
	echo "Loading $@..."

config.h: .config
	@bash tools/generate_config.sh

menuconfig: tools/kconfig-frontends/frontends/mconf/mconf
	tools/kconfig-frontends/frontends/mconf/mconf Kconfig

nconfig: tools/kconfig-frontends/frontends/nconf/nconf
	tools/kconfig-frontends/frontends/nconf/nconf Kconfig

config: tools/kconfig-frontends/frontends/conf/conf
	tools/kconfig-frontends/frontends/conf/conf Kconfig

tools/kconfig-frontends/bin/kconfig-%:
	$(MAKE) -C ./tools/ $(subst tools/kconfig-frontends/bin/,,$@)

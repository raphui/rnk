export

CONFIG := $(wildcard .config)
ifneq ($(CONFIG),)
include $(CONFIG)
endif

ifeq ($(CONFIG_CPU_ARMV7M),y)
ARMV=armv7m
endif

ifeq ($(CONFIG_MACH_STM32),y)
MACH=stm32
endif

ifeq ($(CONFIG_STM32F401),y)
SOC=stm32f401
FAMILY=stm32f4xx
endif

ifeq ($(CONFIG_STM32F407),y)
SOC=stm32f407
FAMILY=stm32f4xx
endif

ifeq ($(CONFIG_STM32F429),y)
SOC=stm32f429
FAMILY=stm32f4xx
endif

ifeq ($(CONFIG_STM32F746),y)
SOC=stm32f746
FAMILY=stm32f7xx
endif

ifeq ($(CONFIG_CPU_ARM_CORTEX_M4),y)
MCPU=cortex-m4
endif

ifeq ($(CONFIG_CPU_ARM_CORTEX_M7),y)
MCPU=cortex-m7
endif

KCONFIG_AUTOHEADER=config.h

ifeq (${MAKELEVEL}, 0)
INCLUDES	+= -I$(KERNEL_BASE)/include
INCLUDES	+= -I$(KERNEL_BASE)/boards
INCLUDES	+= -I$(KERNEL_BASE)/lib
INCLUDES	+= -I$(KERNEL_BASE)/third_party/lib/fdt/include

ifeq ($(CONFIG_TLSF),y)
INCLUDES	+= -I$(KERNEL_BASE)/third_party/lib/tlsf
endif

INCLUDES	+= -include $(KERNEL_BASE)/config.h
ASFLAGS	:= -g $(INCLUDES) -D__ASSEMBLY__ -mcpu=$(MCPU) -mthumb
CFLAGS  :=  -Wall -fno-builtin -ffunction-sections -mcpu=$(MCPU) -mthumb -nostdlib -nostdinc -g $(INCLUDES)
CFLAGS += -MD -MP

ifeq ($(CONFIG_UNWIND),y)
CFLAGS += -funwind-tables
endif

#CFLAGS  :=  -Wall -mlong-calls -fpic -ffunction-sections -mcpu=arm7tdmi -nostdlib -g $(INCLUDES)
#CFLAGS  :=  -Wall -mlong-calls -fpic -ffreestanding -nostdlib -g $(INCLUDES)
LDFLAGS	:= -g $(INCLUDES) -nostartfiles -nostdlib -Wl,-Map=kernel.map#-Wl,--gc-sections

LDSFLAGS := $(INCLUDES)

CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)as
AR := $(CROSS_COMPILE)ar
LD := $(CROSS_COMPILE)ld
OBJCOPY := $(CROSS_COMPILE)objcopy
LDS := $(CROSS_COMPILE)gcc -E -P -C
DTC := dtc

endif

subdirs-y := arch boards boot drivers fs kernel ldscripts lib loader mm third_party utils

linker_files = rnk.lds
dtb = rnk.dtb

ifeq (${MAKELEVEL}, 0)

.PHONY: all clean

all: conf kernel.img

conf:
	@@echo "CP mach-$(MACH)/board-$(FAMILY).h -> board.h"
	@cp boards/mach-$(MACH)/board-$(FAMILY).h boards/board.h
	@ln -f -s $(KERNEL_BASE)/boards/mach-$(MACH)/include $(KERNEL_BASE)/include/mach
	@ln -f -s $(KERNEL_BASE)/arch/arm/include $(KERNEL_BASE)/include/arch
	@ln -f -s $(KERNEL_BASE)/arch/arm/$(ARMV)/include $(KERNEL_BASE)/include/$(ARMV)

cscope:
	@@echo "GEN " $@
	@cscope -b -q -k -R

kernel.elf: conf config.h
	rm -f objects.lst
	rm -f extra_objects.lst
	$(MAKE) -f tools/Makefile.common dir=. all
	$(CC) -T$(linker_files) -o $@ \
		`cat objects.lst | tr '\n' ' '` $(LDFLAGS)
 
include $(wildcard *.d)
 
kernel.img: kernel.elf 
	@@echo "OBJCOPY " $<
	@$(OBJCOPY) kernel.elf -O binary kernel.bin

clean:
	$(MAKE) -f tools/Makefile.common dir=. $@
	$(RM) $(OBJS) kernel.elf kernel.img
	$(RM) boards/board.h
	$(RM) include/arch
	$(RM) include/$(ARMV)
	$(RM) include/mach
 
dist-clean: clean
	$(RM) `find . -name *.d`

tests:
	tools/make_apps.sh

endif

.config:
	echo "ERROR: No config file loaded."
	exit 1

%_defconfig:
	cp arch/${ARCH}/configs/$@ .config
	echo "Loading $@..."

config.h: .config
	@bash tools/generate_config.sh

menuconfig: $(KCONFIG)/kconfig-mconf
	$(KCONFIG)/kconfig-mconf Kconfig

nconfig: $(KCONFIG)/kconfig-nconf
	$(KCONFIG)/kconfig-nconf Kconfig

config: tools/kconfig-frontends/frontends/conf/conf
	tools/kconfig-frontends/frontends/conf/conf Kconfig

tools/kconfig-frontends/bin/kconfig-%:
	$(MAKE) -C ./tools/ $(subst tools/kconfig-frontends/bin/,,$@)

ifndef VERBOSE
.SILENT:
endif

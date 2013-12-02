#SOURCES_ASM := $(wildcard *.S)
#SOURCES_C   := $(wildcard *.c)
 
#OBJS        := $(patsubst %.S,%.o,$(SOURCES_ASM))
#OBJS        += $(patsubst %.c,%.o,$(SOURCES_C))
 
DEPENDFLAGS := -MD -MP
INCLUDES    := -I$(TOOLCHAIN_DIR)arm-unknown-eabi/include
INCLUDES 	+= -I$(KERNEL_BASE)/include
INCLUDES 	+= -I$(KERNEL_BASE)/boards
BASEFLAGS   := -O2 -fpic -pedantic -pedantic-errors -nostdlib
BASEFLAGS   += -nostartfiles -ffreestanding -nodefaultlibs
BASEFLAGS   += -fno-builtin -fomit-frame-pointer -mcpu=arm1176jzf-s
WARNFLAGS   := -Wall -Wextra -Wshadow -Wcast-align -Wwrite-strings
WARNFLAGS   += -Wredundant-decls -Winline
WARNFLAGS   += -Wno-attributes -Wno-deprecated-declarations
WARNFLAGS   += -Wno-div-by-zero -Wno-endif-labels -Wfloat-equal
WARNFLAGS   += -Wformat=2 -Wno-format-extra-args -Winit-self
WARNFLAGS   += -Winvalid-pch -Wmissing-format-attribute
WARNFLAGS   += -Wmissing-include-dirs -Wno-multichar
WARNFLAGS   += -Wredundant-decls -Wshadow
WARNFLAGS   += -Wno-sign-compare -Wswitch -Wsystem-headers -Wundef
WARNFLAGS   += -Wno-pragmas -Wno-unused-but-set-parameter
WARNFLAGS   += -Wno-unused-but-set-variable -Wno-unused-result
WARNFLAGS   += -Wwrite-strings -Wdisabled-optimization -Wpointer-arith
WARNFLAGS   += -Werror
ASFLAGS     := $(INCLUDES) $(DEPENDFLAGS) -D__ASSEMBLY__
CFLAGS      := $(INCLUDES) $(DEPENDFLAGS) $(BASEFLAGS) $(WARNFLAGS)
CFLAGS      += -std=gnu99

OBJS	:= boot/boot.o \
			drivers/uart.o \
			kernel/main.o \
 
all: kernel.img
 
include $(wildcard *.d)
 
kernel.elf: $(OBJS) link-arm-eabi.ld
	$(CROSS_COMPILE)ld $(OBJS) -Tlink-arm-eabi.ld -o $@
 
kernel.img: kernel.elf
	$(CROSS_COMPILE)objcopy kernel.elf -O binary kernel.img
 
clean:
	$(RM) -f $(OBJS) kernel.elf kernel.img
 
dist-clean: clean
	$(RM) -f *.d
 
%.o: %.c Makefile
	$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@
 
%.o: %.S Makefile
	$(CROSS_COMPILE)gcc $(ASFLAGS) -c $< -o $@

export

CONFIG := $(KERNEL_BASE)/.config
ifneq ($(CONFIG),)
include $(CONFIG)
endif

MCPU=cortex-m4

INCLUDES	+= -I$(KERNEL_BASE)/include
INCLUDES	+= -I$(KERNEL_BASE)/boards
INCLUDES	+= -I$(KERNEL_BASE)/lib
INCLUDES	+= -include $(KERNEL_BASE)/config.h

CFLAGS  :=  -Wall -fno-builtin -ffunction-sections -fomit-frame-pointer
CFLAGS += -mcpu=$(MCPU) -mthumb -nostdlib $(INCLUDES)
CFLAGS +=  -fno-common -mlong-calls

ifeq ($(CONFIG_ELF_LOADER),y)
CFLAGS +=  -mword-relocations
else
CFLAGS += -msingle-pic-base -fno-inline -fPIE -mno-pic-data-is-text-relative
endif


LDFLAGS	:= -g $(INCLUDES) -nostartfiles -nostdlib #-Wl,--gc-sections
LDFLAGS += -mcpu=$(MCPU) -mthumb -fno-common

ifeq ($(CONFIG_ELF_LOADER),y)
LDFLAGS += -r -Bsymbolic -mcpu=$(MCPU) -mthumb
else
LDFLAGS += -Wl,--unresolved-symbols=ignore-in-object-files -Wl,--emit-relocs
endif

LD_SCRIPT = $(APPS_BASE)/apps.ld

CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)as
AR := $(CROSS_COMPILE)ar
LD := $(CROSS_COMPILE)ld
RFLAT := $(KERNEL_BASE)/tools/rflat/rflat


TOPTARGETS := all clean

SUBDIRS := sensor tests/mtd_tests tests/mutex_tests tests/queue_tests tests/sem_tests tests/syscall_tests tests/thread_tests tests/timer_tests

$(TOPTARGETS): $(SUBDIRS)

$(SUBDIRS):
	        $(MAKE) -C $@ $(MAKECMDGOALS)

print-%:
	@echo $*=$($*)

.PHONY: $(TOPTARGETS) $(SUBDIRS)

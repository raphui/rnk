export

CONFIG := $(wildcard .config)
ifneq ($(CONFIG),)
include $(CONFIG)
endif

subdirs-$(CONFIG_STATIC_APPS) := apps
subdirs-y += arch
subdirs-y += boards
subdirs-y += boot
subdirs-y += drivers
subdirs-y += fs
subdirs-y += kernel
subdirs-y += ldscripts
subdirs-y += lib
subdirs-y += loader
subdirs-y += mm
subdirs-y += third_party
subdirs-y += utils

linker_files = rnk.lds
dtb = rnk.dtb

ifeq (${MAKELEVEL}, 0)

CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)as
AR := $(CROSS_COMPILE)ar
LD := $(CROSS_COMPILE)ld
OBJCOPY := $(CROSS_COMPILE)objcopy
LDS := $(CROSS_COMPILE)gcc -E -P -C
DTC := $(KERNEL_BASE)/tools/dtc
CPP := cpp
MKDIR := mkdir -p
PYTHON := python3
RFLAT := $(KERNEL_BASE)/tools/rflat/rflat

DT_PARSER=$(KERNEL_BASE)/tools/dt_parser/dt_parser.py

ifdef DEBUGMAKE
else
PREFIX=@
endif

.PHONY: apps all clean

all: config.h
	$(PREFIX)rm -f objects.lst
	$(PREFIX)rm -f extra_objects.lst
	$(PREFIX)$(MAKE) --no-print-directory -C tools -f Makefile.kernel dir=. all
 
clean:
	$(PREFIX)$(MAKE) --no-print-directory -C tools -f Makefile.kernel dir=. $@
	$(PREFIX)$(MAKE) --no-print-directory -C tools -f Makefile.apps dir=. $@
 
dist-clean: clean
	$(PREFIX)$(RM) `find . -name *.d`
	$(PREFIX)$(RM) `find . -name *.tmp`

%_defconfig:
	$(PREFIX)cp arch/${ARCH}/configs/$@ .config
	echo "Loading $@..."

config.h: .config
	$(PREFIX)bash tools/generate_config.sh

menuconfig: $(KCONFIG)/kconfig-mconf
	$(PREFIX)$(KCONFIG)/kconfig-mconf Kconfig

nconfig: $(KCONFIG)/kconfig-nconf
	$(PREFIX)$(KCONFIG)/kconfig-nconf Kconfig

config: tools/kconfig-frontends/frontends/conf/conf
	$(PREFIX)tools/kconfig-frontends/frontends/conf/conf Kconfig

tools/kconfig-frontends/bin/kconfig-%:
	$(PREFIX)$(MAKE) -C ./tools/ $(subst tools/kconfig-frontends/bin/,,$@)

cscope:
	$(PREFIX)echo "GEN " $@
	$(PREFIX)cd $(KERNEL_BASE); cscope -b -q -k -R

%_tests:
	$(PREFIX)$(MAKE) --no-print-directory -C tools -f Makefile.apps dir=$(APPS_BASE)/tests/$@ app=$@ all

%:
	$(PREFIX)$(MAKE) --no-print-directory -C tools -f Makefile.apps dir=$(APPS_BASE)/$@ app=$@ all

endif

ifndef VERBOSE
.SILENT:
endif

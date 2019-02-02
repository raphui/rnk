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
DTC := dtc
CPP := cpp
MKDIR := mkdir -p
PYTHON := python3
RFLAT := $(KERNEL_BASE)/tools/rflat/rflat

DT_PARSER=$(KERNEL_BASE)/tools/dt_parser/dt_parser.py


.PHONY: apps all clean

all: config.h
	rm -f objects.lst
	rm -f extra_objects.lst
	$(MAKE) -C tools -f Makefile.kernel dir=. all
 
clean:
	$(MAKE) -C tools -f Makefile.kernel dir=. $@
	$(MAKE) -C tools -f Makefile.apps dir=. $@
 
dist-clean: clean
	$(RM) `find . -name *.d`
	$(RM) `find . -name *.tmp`

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

cscope:
	@@echo "GEN " $@
	@cd $(KERNEL_BASE); cscope -b -q -k -R

%_tests:
	$(MAKE) -C tools -f Makefile.apps dir=$(APPS_BASE)/tests/$@ app=$@ all

%:
	$(MAKE) -C tools -f Makefile.apps dir=$(APPS_BASE)/$@ app=$@ all

endif

ifndef VERBOSE
.SILENT:
endif

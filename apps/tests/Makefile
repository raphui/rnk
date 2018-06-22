TOPTARGETS := all clean

SUBDIRS := $(wildcard */.)

$(TOPTARGETS):
	$(SUBDIRS):
	        $(MAKE) -C $@ $(MAKECMDGOALS)

.PHONY: $(TOPTARGETS) $(SUBDIRS)

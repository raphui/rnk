TARGET_NAME := mtd_test

OBJS := mtd_test.o

MCPU=cortex-m4

all: $(TARGET_NAME)

clean:
	$(RM) $(TARGET_NAME)
	$(RM) $(OBJS)
	
$(TARGET_NAME): $(OBJS) 
	@@echo "LD " $@
	@$(CC) $(LDFLAGS) -T$(LD_SCRIPT) -o $@ $^
ifeq ($(CONFIG_RFLAT_LOADER), y)
	$(RFLAT) $(TARGET_NAME) $(TARGET_NAME).bin
endif

%.o: %.c
	@@echo "CC " $<
	@$(CC) $(CFLAGS) -c $< -o $@

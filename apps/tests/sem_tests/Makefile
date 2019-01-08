TARGET_NAME := sem_test

OBJS := sem_test.o

all: $(TARGET_NAME)

clean:
	$(RM) $(TARGET_NAME)
	$(RM) $(OBJS)
	
$(TARGET_NAME): $(OBJS) 
	@@echo "LD " $@
	@$(CC) $(LDFLAGS) -T$(LD_SCRIPT) -o $@ $^

%.o: %.c
	@@echo "CC " $<
	@$(CC) $(CFLAGS) -c $< -o $@

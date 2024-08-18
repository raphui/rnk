TARGET := mx_tracker
INCLUDES_LIBS_HEADERS := -I/usr/local/include/ -I/usr/include/
GCC := gcc
DFLAGS := -g -O0 -fno-omit-frame-pointer -pipe -Wall

SRCF= $(wildcard *.c)
OBJF= $(SRCF:.c=.o)

%.o: %.c
	$(GCC) $(DFLAGS) $(INCLUDES_LIBS_HEADERS) -c -fmessage-length=0 -o $@ $<

test: $(OBJF)
	$(GCC) $(OBJF) -o $(TARGET) $(LIBS)

all: $(OBJF)
	$(GCC) $(OBJF) -o $(TARGET) $(LIBS)
	rm -f $(OBJF)
	rm -f *.o

clean:
	rm -f $(OBJF) rm -f $(TARGET)

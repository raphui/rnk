#include <asm.h>

.syntax unified
.thumb

.thumb_func
ENTRY(lowlevel_delay)
	subs r0, r0, #1	
	cmp r0, #0
	bne lowlevel_delay
	bx lr

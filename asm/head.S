#include <asm.h>

ENTRY( get_value_cp )

	mrc p15, 0, r0, c1, c0, 0
	mov pc, lr

END( get_value_cp )

#include <asm.h>

#define INSTR_CODE_SVC		(0xDF00)
#define INSTR_CODE_BX_LR	(0x4770)

.syntax unified
.thumb

.thumb_func
ENTRY(svc_noarg)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_arg1)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_arg2)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_arg3)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_call)
	push {lr}
	svc #0
	pop {pc}

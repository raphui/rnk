.global svc_activate_context
svc_activate_context:
	mov	sp, r0
	ldmia	sp!, {r0-r3} /* r0: xPSR, r1: PC, r2: LR, r3: 0 */
	mov	lr, r2
	msr	CPSR_c, #0x10
	mov	pc, r1

.global svc_switch_context
svc_switch_context:
	mov	pc, lr

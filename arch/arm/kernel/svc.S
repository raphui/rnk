#define ARM_MODE_USR	0x10
#define ARM_MODE_FIQ	0x11
#define ARM_MODE_IRQ    0x12
#define ARM_MODE_SVC    0x13
#define ARM_MODE_ABT    0x17
#define ARM_MODE_SYS	0x1F

#define I_BIT            0x80
#define F_BIT            0x40

.global svc_create_context
svc_create_context:
	/* r0: start_task, r1: func, r2: lr, r3: frame pointer */
	/* Switch to sys mode */
	msr	CPSR_c, #ARM_MODE_SYS | I_BIT | F_BIT
	mov	sp, r0
	stmdb	sp!, {r1-r3}
	msr	CPSR_c, #ARM_MODE_SVC
	mov	pc, lr

.global svc_activate_context
svc_activate_context:
	mov	sp, r0
	ldmia	sp!, {r0-r3} /* r0: xPSR, r1: PC, r2: LR, r3: 0 */
	mov	lr, r2
	msr	CPSR_c, #ARM_MODE_USR
	mov	pc, r1

.global svc_switch_context
svc_switch_context:
	mov	pc, lr

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
	/* r1: start_stack, r2: func, r3: lr, r4: frame pointer */
	/* push sp and lr on stack for first context switch */
/*
	------------------------- SP Origin
	|	R0		|	
	-------------------------	
	|	R1		|
	-------------------------
	|	R2		|	|
	-------------------------	|
	|	R3		|	|	Desc
	-------------------------	|
	|	R4		|	|
	-------------------------       v
	|	R5		|	
	-------------------------
	|	R6		|
	-------------------------
	|	R7		|
	-------------------------
	|	R8		|
	-------------------------
	|	R9		|
	-------------------------
	|	R10		|
	-------------------------
	|	R11		|
	-------------------------
	|	R12		|
	-------------------------
	|	LR		|
	-------------------------
	|	SPSR		|
	-------------------------
	|	PC		|
	------------------------- SP
*/
	mov	r6, #0x0
	mov	r5, #ARM_MODE_USR
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r6}
	stmfd	r1!, {r3}
	stmfd	r1!, {r5}
	stmfd	r1!, {r2}
	add	r0, r0, #0x34
	str	r1, [r0]
	mov	pc, lr

/*
	mov	r6, r1
	add	r6, r6, #0x34
	str	r1, [r6]
	add	r6, r6, #0x4
	str	r3, [r6]
	add	r6, r6, #0x4
	str	r2, [r6]
	mov	r5, r1
	stmdb	r5!, {r1-r4}
	mov	pc, lr
*/

.global svc_activate_context
svc_activate_context:
	/* r0 point to SP */
	add	r0, r0, #0x34
	msr	CPSR_c, #ARM_MODE_SYS | I_BIT | F_BIT
	ldr	sp, [r0]
	/* r0 point to the beginning of the structure */
	sub	r0, r0, #0x34
	ldmfd	sp!, {r0}
	ldmfd	sp!, {r1}
	ldmfd	sp!, {r2}
	ldmfd	sp!, {r3}/* r0: PC, r1: SPSR, r2: LR, r3: 0 */
	mov	lr, r2
	msr	CPSR_c, #ARM_MODE_USR | F_BIT
	mov	pc, r0

.global svc_switch_context
svc_switch_context:
	/* r0: current reg structure */
	/* r1: new reg structure */

	/* Save SVC context */
	stmdb	sp!, {r0-r12, lr}

	msr	CPSR_c,	#ARM_MODE_SYS | I_BIT | F_BIT

	/* Save current task sp in structure */
	add	r0, r0, #0x34
	str	sp, [r0]

	/* Get new current task sp from structure */
	add	r1, r1, #0x34
	ldr	sp, [r1]

	msr	CPSR_c, #ARM_MODE_SVC | I_BIT | F_BIT

	ldmia	sp!, {r0-r12, lr}
	mov	pc, lr

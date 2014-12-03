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
	/* r0: PC, r1: SPSR, r2: LR, r3: 0 */
	ldmfd	sp!, {r3}
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

.global svc_save_user_context
svc_save_user_context:
	/* Save User mode context */
	msr	CPSR_c, #ARM_MODE_SYS | I_BIT | F_BIT
	stmfd	sp!, {r0}
	stmfd	sp!, {r1}
	stmfd	sp!, {r2}
	stmfd	sp!, {r3}
	stmfd	sp!, {r4}
	stmfd	sp!, {r5}
	stmfd	sp!, {r6}
	stmfd	sp!, {r7}
	stmfd	sp!, {r8}
	stmfd	sp!, {r9}
	stmfd	sp!, {r10}
	stmfd	sp!, {r11}
	stmfd	sp!, {r12}
	stmfd	sp!, {lr}
	mov	r7, sp

	/* Restore mode */
	msr     CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT

	stmfd   r7!, {r8}
        stmfd   r7!, {r10}

/*	Update User mode stack */
	msr	CPSR_c, #ARM_MODE_SYS | I_BIT | F_BIT
	mov	sp, r7

	msr     CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT
	mov	pc, lr

.global svc_get_user_context
svc_get_user_context:
	ldr	r1, =task_switching
	ldr	r2, [r1]
	cmp	r2, #0
	beq	skip_user_restore
	mov	r2, #0
	str	r2, [r1]

	/* Get User mode context */
	msr	CPSR_c, #ARM_MODE_SYS | I_BIT | F_BIT

	ldr	r1, =.get_pc
	ldmfd	sp!, {lr}
	str	lr, [r1]
	ldr	r1, =.get_spsr
	ldmfd	sp!, {lr}
	str	lr, [r1]

	ldmfd	sp!, {lr}
	ldmfd	sp!, {r12}
	ldmfd	sp!, {r11}
	ldmfd	sp!, {r10}
	ldmfd	sp!, {r9}
	ldmfd	sp!, {r8}
	ldmfd	sp!, {r7}
	ldmfd	sp!, {r6}
	ldmfd	sp!, {r5}
	ldmfd	sp!, {r4}
	ldmfd	sp!, {r3}
	ldmfd	sp!, {r2}
	ldmfd	sp!, {r1}
	ldmfd	sp!, {r0}

	/* Restore mode */
	msr     CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT

	ldr	r8, =.get_spsr
	ldr	r8, [r8]
	msr	CPSR_c, r8
	ldr	r8, =.get_pc
	ldr	r8, [r8]
	mov	pc, r8

skip_user_restore:
	mov	pc, lr

.get_pc:
	.word 0

.get_spsr:
	.word 0


#include <printk.h>
#include <errno.h>
#include <stddef.h>
#include <utils.h>

#include <symbols.h>

/* Dummy functions to avoid linker complaints */
void __aeabi_unwind_cpp_pr0(void)
{
};

void __aeabi_unwind_cpp_pr1(void)
{
};

void __aeabi_unwind_cpp_pr2(void)
{
};

struct unwind_ctrl_block {
	unsigned long vrs[16];		/* virtual register set */
	unsigned long *insn;		/* pointer to the current instructions word */
	int entries;			/* number of entries left to interpret */
	int byte;			/* current byte number in the instructions word */
};

enum regs {
	FP = 11,
	SP = 13,
	LR = 14,
	PC = 15
};

/* Unwind reason code according the the ARM EABI documents */
enum unwind_reason_code {
	URC_OK = 0,			/* operation completed successfully */
	URC_CONTINUE_UNWIND = 8,
	URC_FAILURE = 9			/* unspecified failure of some kind */
};

struct unwind_idx {
	unsigned long addr;
	unsigned long insn;
};

struct stackframe {
	unsigned long fp;
	unsigned long sp;
	unsigned long lr;
	unsigned long pc;
};

#define THREAD_SIZE               8192

extern struct unwind_idx __exidx_start[];
extern struct unwind_idx __exidx_end[];
//extern unsigned long __exidx_start[];
//extern unsigned long __exidx_end[];
extern unsigned long _text[];
extern unsigned long _etext[];



/* Convert a prel31 symbol to an absolute address */
#define prel31_to_addr(ptr)				\
({							\
	/* sign-extend to 32 bits */			\
	long offset = (((long)*(ptr)) << 1) >> 1;	\
	(unsigned long)(ptr) + offset;			\
})

static inline int is_kernel_text(unsigned long addr)
{
	if ((addr >= (unsigned long)_text && addr <= (unsigned long)_etext))
		return 1;
	return 0;
}

void dump_backtrace_entry(unsigned long where, unsigned long from, unsigned long frame)
{
	char *sym_where = symbol_get_name((unsigned int)where);
	char *sym_from = symbol_get_name((unsigned int)from);

	printk("[<%x>] (%s) from [<%x>] (%s)\n", where, sym_where, from, sym_from);
}

/*
 * Binary search in the unwind index. The entries entries are
 * guaranteed to be sorted in ascending order by the linker.
 */
static struct unwind_idx *search_index(unsigned long addr,
				       struct unwind_idx *first,
				       struct unwind_idx *last)
{
	debug_printk("%s(%x, %p, %p)\n", __func__, addr, first, last);

	if (addr > last->addr)
		return last;
	else if (addr >= first->addr) {
		error_printk("unwind: Unknown symbol address %x\n", addr);
		return NULL;
	}

	while (first < last - 1) {
		struct unwind_idx *mid = first + ((last - first + 1) >> 1);

		if (addr < mid->addr)
			last = mid;
		else
			first = mid;
	}

	return first;
}

static struct unwind_idx *unwind_find_idx(unsigned long addr)
{
	struct unwind_idx *idx = NULL;

	debug_printk("%s(%x)\n", __func__, addr);

	if (is_kernel_text(addr))
		/* main unwind table */
		idx = search_index(addr, __exidx_start,
				   __exidx_end - 1);
	else {
		/* module unwinding not supported */
	}

	debug_printk("%s: idx = %p\n", __func__, idx);
	return idx;
}

static unsigned long unwind_get_byte(struct unwind_ctrl_block *ctrl)
{
	unsigned long ret;

	if (ctrl->entries <= 0) {
		error_printk("unwind: Corrupt unwind table\n");
		return 0;
	}

	ret = (*ctrl->insn >> (ctrl->byte * 8)) & 0xff;

	if (ctrl->byte == 0) {
		ctrl->insn++;
		ctrl->entries--;
		ctrl->byte = 3;
	} else
		ctrl->byte--;

	return ret;
}

/*
 * Execute the current unwind instruction.
 */
static int unwind_exec_insn(struct unwind_ctrl_block *ctrl)
{
	unsigned long insn = unwind_get_byte(ctrl);

	debug_printk("%s: insn = %x\n", __func__, insn);

	if ((insn & 0xc0) == 0x00)
		ctrl->vrs[SP] += ((insn & 0x3f) << 2) + 4;
	else if ((insn & 0xc0) == 0x40)
		ctrl->vrs[SP] -= ((insn & 0x3f) << 2) + 4;
	else if ((insn & 0xf0) == 0x80) {
		unsigned long mask;
		unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
		int load_sp, reg = 4;

		insn = (insn << 8) | unwind_get_byte(ctrl);
		mask = insn & 0x0fff;
		if (mask == 0) {
			error_printk("unwind: 'Refuse to unwind' instruction %x\n",
				   insn);
			return -URC_FAILURE;
		}

		/* pop R4-R15 according to mask */
		load_sp = mask & (1 << (13 - 4));
		while (mask) {
			if (mask & 1)
				ctrl->vrs[reg] = *vsp++;
			mask >>= 1;
			reg++;
		}
		if (!load_sp)
			ctrl->vrs[SP] = (unsigned long)vsp;
	} else if ((insn & 0xf0) == 0x90 &&
		   (insn & 0x0d) != 0x0d)
		ctrl->vrs[SP] = ctrl->vrs[insn & 0x0f];
	else if ((insn & 0xf0) == 0xa0) {
		unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
		int reg;

		/* pop R4-R[4+bbb] */
		for (reg = 4; reg <= 4 + (insn & 7); reg++)
			ctrl->vrs[reg] = *vsp++;
		if (insn & 0x80)
			ctrl->vrs[14] = *vsp++;
		ctrl->vrs[SP] = (unsigned long)vsp;
	} else if (insn == 0xb0) {
		if (ctrl->vrs[PC] == 0)
			ctrl->vrs[PC] = ctrl->vrs[LR];
		/* no further processing */
		ctrl->entries = 0;
	} else if (insn == 0xb1) {
		unsigned long mask = unwind_get_byte(ctrl);
		unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
		int reg = 0;

		if (mask == 0 || mask & 0xf0) {
			error_printk("unwind: Spare encoding %04lx\n",
			       (insn << 8) | mask);
			return -URC_FAILURE;
		}

		/* pop R0-R3 according to mask */
		while (mask) {
			if (mask & 1)
				ctrl->vrs[reg] = *vsp++;
			mask >>= 1;
			reg++;
		}
		ctrl->vrs[SP] = (unsigned long)vsp;
	} else if (insn == 0xb2) {
		unsigned long uleb128 = unwind_get_byte(ctrl);

		ctrl->vrs[SP] += 0x204 + (uleb128 << 2);
	} else {
		error_printk("unwind: Unhandled instruction %02lx\n", insn);
		return -URC_FAILURE;
	}

	debug_printk("%s: fp = %x sp = %x lr = %x pc = %x\n", __func__,
		 ctrl->vrs[FP], ctrl->vrs[SP], ctrl->vrs[LR], ctrl->vrs[PC]);

	return URC_OK;
}

/*
 * Unwind a single frame starting with *sp for the symbol at *pc. It
 * updates the *pc and *sp with the new values.
 */
int unwind_frame(struct stackframe *frame)
{
	unsigned long high, low;
	struct unwind_idx *idx;
	struct unwind_ctrl_block ctrl;

	/* only go to a higher address on the stack */
	low = frame->sp;
	high = ALIGN(low, THREAD_SIZE) + THREAD_SIZE;

	debug_printk("%s(pc = %x lr = %x sp = %x)\n", __func__,
		 frame->pc, frame->lr, frame->sp);

	if (!is_kernel_text(frame->pc))
		return -URC_FAILURE;

	idx = unwind_find_idx(frame->pc);
	if (!idx) {
		error_printk("unwind: Index not found %x\n", frame->pc);
		return -URC_FAILURE;
	}

	ctrl.vrs[FP] = frame->fp;
	ctrl.vrs[SP] = frame->sp;
	ctrl.vrs[LR] = frame->lr;
	ctrl.vrs[PC] = 0;

	if (idx->insn == 1)
		/* can't unwind */
		return -URC_FAILURE;
	else if ((idx->insn & 0x80000000) == 0)
		/* prel31 to the unwind table */
		ctrl.insn = (unsigned long *)prel31_to_addr(&idx->insn);
	else if ((idx->insn & 0xff000000) == 0x80000000)
		/* only personality routine 0 supported in the index */
		ctrl.insn = &idx->insn;
	else {
		error_printk("unwind: Unsupported personality routine %x in the index at %p\n",
			   idx->insn, idx);
		return -URC_FAILURE;
	}

	/* check the personality routine */
	if ((*ctrl.insn & 0xff000000) == 0x80000000) {
		ctrl.byte = 2;
		ctrl.entries = 1;
	} else if ((*ctrl.insn & 0xff000000) == 0x81000000) {
		ctrl.byte = 1;
		ctrl.entries = 1 + ((*ctrl.insn & 0x00ff0000) >> 16);
	} else {
		error_printk("unwind: Unsupported personality routine %x at %p\n",
			   *ctrl.insn, ctrl.insn);
		return -URC_FAILURE;
	}

	while (ctrl.entries > 0) {
		int urc = unwind_exec_insn(&ctrl);
		if (urc < 0)
			return urc;
		if (ctrl.vrs[SP] < low || ctrl.vrs[SP] >= high)
			return -URC_FAILURE;
	}

	if (ctrl.vrs[PC] == 0)
		ctrl.vrs[PC] = ctrl.vrs[LR];

	/* check for infinite loop */
	if (frame->pc == ctrl.vrs[PC])
		return -URC_FAILURE;

	frame->fp = ctrl.vrs[FP];
	frame->sp = ctrl.vrs[SP];
	frame->lr = ctrl.vrs[LR];
	frame->pc = ctrl.vrs[PC];

	return URC_OK;
}

void unwind_backtrace(unsigned int fp, unsigned int sp, unsigned int lr, unsigned pc)
{
	struct stackframe frame;

	debug_printk("%s(%x, %x, %x, %x)\n", __func__, fp, sp, lr, pc);

	frame.fp = fp;
	frame.sp = sp;
	frame.lr = lr;
	/* PC might be corrupted, use LR in that case. */
	frame.pc = is_kernel_text(pc)
		 ? pc : lr;

	while (1) {
		int urc;
		unsigned long where = frame.pc;

		urc = unwind_frame(&frame);
		if (urc < 0)
			break;
		dump_backtrace_entry(where, frame.pc, frame.sp - 4);
	}
}

int unwind_init(void)
{
	struct unwind_idx *idx;

	/* Convert the symbol addresses to absolute values */
	for (idx = __exidx_start; idx < __exidx_end; idx++)
		idx->addr = prel31_to_addr(&idx->addr);

	debug_printk("%x %x %x %x\r\n", __exidx_start, __exidx_end, _text, _etext);

	return 0;
}

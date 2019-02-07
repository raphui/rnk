
#include <kernel/printk.h>
#include <string.h>
#include <errno.h>
#include <rflat/rflat.h>
#include <symbols.h>
#include <mm/mm.h>
#include <unistd.h>
#include <kernel/thread.h>

#define R_ARM_NONE		0	/* Nothing */
#define R_ARM_ABS32		2	/* (S + A) | T */
#define R_ARM_THM_CALL		10	/* ((S + A) | T) – P */
#define R_ARM_GOT_BREL		26	/* GOT(S) + A - GOT_ORG */
#define R_ARM_CALL		28	/* ((S + A) | T) – P */
#define R_ARM_JUMP24		29	/* ((S + A) | T) – P */
#define R_ARM_V4BX		40	/* Nothing */
#define R_ARM_PREL31		42	/* (S + A) | T */
#define R_ARM_THM_MOVW_ABS_NC	47	/* (S + A) | T */
#define R_ARM_THM_MOVT_ABS	48	/* S + A */

struct rflat {
	struct rflat_header *header;
	unsigned char *text_seg;
	unsigned char *rodata_seg;
	unsigned char *bss_seg;
	unsigned char *data_seg;
	unsigned char *got;
	int reloc_entries;
	struct rflat_app_header *app_header;
};

struct rflat_reloc_info {
	unsigned int offset;
	unsigned int info;
	unsigned int sym_val;
	unsigned int hash;
};

static struct rflat_app_header *current_app_header;

static void *rflat_alloc(int size)
{
#ifdef CONFIG_USER
	void *mem;

	umalloc(size, &mem);

	return mem;
#else
	return kmalloc(size);
#endif
}

static int rflat_alloc_segments(struct rflat_header *header, struct rflat *ctx)
{
	int ret = 0;
	int offset;
	unsigned char *ptr;

	ptr = rflat_alloc(header->bss_size + header->data_size);
	if (!ptr) {
		error_printk("failed to allocate bss and data\n");
		return -ENOMEM;
	}

	memset(ptr, 0, header->bss_size);

	offset = (int)((int)header + RFLAT_GET_SEG_ADDR(header->data_offset));
	memcpy(ptr + header->bss_size, (void *)offset, header->data_size);

	ctx->bss_seg = ptr;
	ctx->data_seg = ptr + header->bss_size;

	ptr = rflat_alloc(header->got_size);
	if (!ptr) {
		error_printk("failed to allocate GOT\n");
		return -ENOMEM;
	}

	ctx->got = ptr;

	ptr = rflat_alloc(sizeof(struct rflat_app_header));
	if (!ptr) {
		error_printk("failed to allocate rflat app header\n");
		return -ENOMEM;
	}

	ctx->app_header = (struct rflat_app_header *)ptr;

	return ret;
}

static int rflat_find_segment_addr(struct rflat *ctx, unsigned int idx)
{
	int ret = 0;

	if (idx == RFLAT_GET_SEG_INDEX((unsigned int)ctx->header->bss_offset))
		ret = (int)ctx->bss_seg;
	else if (idx == RFLAT_GET_SEG_INDEX((unsigned int)ctx->header->data_offset))
		ret = (int)ctx->data_seg;
	else if (idx == RFLAT_GET_SEG_INDEX((unsigned int)ctx->header->rodata_offset))
		ret = (int)ctx->rodata_seg;
	else if (idx == RFLAT_GET_SEG_INDEX((unsigned int)ctx->header->text_offset))
		ret = (int)ctx->text_seg;
	else
		ret = 0;

	return ret;
}

static int rflat_find_symbol(struct rflat *ctx, struct rflat_reloc_info *rel)
{
	int ret = 0;

	ret = rflat_find_segment_addr(ctx, RFLAT_GET_SEG_INDEX(rel->sym_val));
	if (ret)
		goto out;

	ret = symbol_get_addr_by_hash(rel->hash);

out:
	return ret;
}

static int rflat_reloc(struct rflat *ctx, unsigned int seg_addr, struct rflat_reloc_info *rel)
{
	int *ref = (int *)(seg_addr + rel->offset);
	int func = 0;
	int ret = 0;
	int t;

	if (ELF32_R_SYM(rel->info) != SHN_UNDEF) {
		func = rflat_find_symbol(ctx, rel);

		debug_printk("\t- target: 0x%x\n", seg_addr);
		debug_printk("\t- reloff in target: 0x%x\n", ref);
		debug_printk("\t- func: 0x%x\n", func);

	}

	t = func & 0x1;

	debug_printk("\t- %s instruction\n", t ? "Thumb" : "ARM");
	debug_printk("\t- before reloc: 0x%x\n", *ref);

	switch (ELF32_R_TYPE(rel->info)) {
	case R_ARM_GOT_BREL:
		if (!RFLAT_GET_SEG_INDEX(rel->sym_val))
			*(unsigned int *)(ctx->got + *ref) = func | t;
		else
			*(unsigned int *)(ctx->got + *ref) = (func | t) + RFLAT_GET_SEG_ADDR(rel->sym_val);//*ref;

		break;
	default:
		error_printk("unknown/unsupported reloc type\n");
		return -EINVAL;
	}

	debug_printk("\t- after reloc: 0x%x\n", *ref);

	return ret;
}

static int rflat_parse_reloc(struct rflat_header *header, struct rflat *ctx, struct rflat_reloc_info *reltab)
{
	int i;
	int ret = 0;

	for (i = 0; i < ctx->reloc_entries; i++) {
		rflat_reloc(ctx, (unsigned int)ctx->text_seg, &reltab[i]);
	}

	return ret;
}

static int rflat_load(char *rflat_data)
{
	int ret = 0;
	struct rflat ctx;
	struct rflat_header *header = (struct rflat_header *)rflat_data;
	struct rflat_reloc_info *reltab = (struct rflat_reloc_info *)(rflat_data + header->reloc_infos);

	if (header->magic != RFLAT_MAGIC) {
		error_printk("not a valid rflat binary\n");
		return -EINVAL;
	}

	ctx.header = header;
	ctx.text_seg = (unsigned char*)(rflat_data + RFLAT_GET_SEG_ADDR(header->text_offset));
	ctx.rodata_seg = (unsigned char*)(rflat_data + RFLAT_GET_SEG_ADDR(header->rodata_offset));
	ctx.reloc_entries = header->reloc_n;

	ret = rflat_alloc_segments(header, &ctx);
	if (ret < 0) {
		error_printk("failed to alloc rflat segments\n");
		goto err;
	}

	ret = rflat_parse_reloc(header, &ctx, reltab);
	if (ret < 0) {
		error_printk("failed to parse rflat reloc\n");
		goto err;
	}

	ctx.app_header->addr = (int)(rflat_data + header->entry_point);
	ctx.app_header->size = RFLAT_GET_SEG_ADDR(header->data_offset) - RFLAT_GET_SEG_ADDR(header->text_offset);
	ctx.app_header->got_loc = (int)ctx.got;

	current_app_header = ctx.app_header;
err:
	return ret;
}

struct rflat_app_header *rflat_get_app_header(void)
{
	return current_app_header;
}

int rflat_exec(char *rflat_data)
{
	int ret = 0;
	struct rflat_app_header *app_header;

	ret = rflat_load(rflat_data);
	if (ret < 0) {
		error_printk("failed to load rflat binary\n");
		goto out;
	}

	app_header = rflat_get_app_header();
	ret = app_header->addr;

out:
	return ret;
}

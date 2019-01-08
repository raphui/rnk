#ifndef RFLAT_H
#define RFLAT_H

#include <elf/elf.h>

struct rflat_header {
	unsigned int magic;
	unsigned int reloc_status;
	unsigned int entry_point;
	unsigned int text_offset;
	unsigned int rodata_offset;
	unsigned int bss_offset;
	unsigned int data_offset;
	unsigned int bss_size;
	unsigned int data_size;
	unsigned int got_size;
	unsigned int reloc_infos;
	unsigned int reloc_n;
};

struct rflat_app_header {
	unsigned int addr;
	unsigned int got_loc;
};

#define RFLAT_MAGIC	0x52464C41

#define RFLAT_SET_SEG_INDEX(x, idx)	(x | (idx << 24))
#define RFLAT_GET_SEG_INDEX(x)	((x & (0xFF000000)) >> 24)
#define RFLAT_GET_SEG_ADDR(x)	(x & 0x00FFFFFF)


int rflat_exec(char *rflat_data);
struct rflat_app_header *rflat_get_app_header(void);

#endif /* RFLAT_H */

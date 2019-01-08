#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <elf.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define RFLAT_MAGIC	0x52464C41

#define RFLAT_SET_SEG_INDEX(x, idx)	(x | (idx << 24))
#define RFLAT_GET_SEG_INDEX(x)	(x & ((0xFF000000) >> 24))
#define RFLAT_GET_SEG_ADDR(x)	(x & 0x00FFFFFF)

#define DEBUG_PRINTF	0

#define debug_printf(...) do{ if(DEBUG_PRINTF){ printf(__VA_ARGS__);}}while(0)

#define ALIGN(x, a)	(((x) + (a) - 1) & ~((a) - 1))

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

struct rflat_reloc_infos {
	unsigned int offset;
	unsigned int info;
	unsigned int sym_val;
	unsigned int hash;
};

static char *buff;
static int size;
static int text_offset;
static int text_size;
static int data_offset;
static int data_size;
static int rodata_offset;
static int rodata_size;
static int section_size;
static int text_idx;
static int rodata_idx;
static int data_idx;
static int bss_idx;
static Elf32_Ehdr *ehdr;
static Elf32_Shdr *symtab;
static Elf32_Shdr *strtab;

static struct rflat_reloc_infos *reltab;

static Elf32_Shdr *elf_get_section(int num)
{
	return (Elf32_Shdr *)(buff + ehdr->e_shoff + num * section_size);
}

static unsigned int hash_str(char *str)
{
	unsigned int hash = 5381;
	int c;

	while ((c = *str++))
		hash = ((hash << 5) + hash) + c; /*  hash * 33 + c */

	return hash;
}

static char *elf_get_sym(Elf32_Sym *sym)
{
       return buff + strtab->sh_offset + sym->st_name;
}

static int elf_extract_reloc_infos(Elf32_Shdr *shdr, struct rflat_header *header)
{
	int i, j;
	int n_rel;
	int ret = 0;
	char *str;
	Elf32_Shdr *section;
	Elf32_Sym *sym;
	Elf32_Rel *rel;

	debug_printf("[+] dumping section needed relocation: \n");
	for (i = 0; i < ehdr->e_shnum; i++) {
		section = elf_get_section(i);
		str = buff + shdr->sh_offset + section->sh_name;

		if (section->sh_type == SHT_REL) {
			debug_printf("[!] %s\n", str);
			debug_printf("rel infos: \n");
			debug_printf("\toffset\t\tinfo\t\ttype\t\tname\t\tvalue\n");

			n_rel = section->sh_size / section->sh_entsize;

			reltab = (struct rflat_reloc_infos *)malloc(n_rel * sizeof(*reltab));

			for (j = 0; j < n_rel; j++) {
				rel = (Elf32_Rel *)(buff + section->sh_offset + j * section->sh_entsize);

				sym = (Elf32_Sym *)(buff + symtab->sh_offset + ELF32_R_SYM(rel->r_info) * symtab->sh_entsize);

				debug_printf("\t%#x\t\t%#x\t\t%#x\t\t%s\t\t%x\t\t%d\n", rel->r_offset, rel->r_info, ELF32_R_TYPE(rel->r_info), elf_get_sym(sym), sym->st_value, sym->st_shndx);

				reltab[j].offset = rel->r_offset;
				reltab[j].info = rel->r_info;
				reltab[j].sym_val = RFLAT_SET_SEG_INDEX(sym->st_value, sym->st_shndx) - elf_get_section(sym->st_shndx)->sh_addr;
				reltab[j].hash = hash_str(elf_get_sym(sym));
			}
		}
	}

	header->reloc_n = n_rel;

	return ret;
}

static int elf_parse(Elf32_Shdr *shdr, struct rflat_header *header, char *buff)
{
	int i = 0;
	char *str;
	Elf32_Shdr *section;

	for (i = 0; i < ehdr->e_shnum; i++) {
		section = elf_get_section(i);
		str = buff + shdr->sh_offset + section->sh_name;

		if (!strcmp(str, ".text")) {
			text_offset = section->sh_offset;
			text_size = ALIGN(section->sh_size, 4);
			text_idx = i;
		}

		if (!strcmp(str, ".rodata")) {
			rodata_offset = section->sh_offset;
			rodata_size = ALIGN(section->sh_size, 4);
			rodata_idx = i;
		}

		if (!strcmp(str, ".bss")) {
			header->bss_offset = 0;
			header->bss_size = section->sh_size;
			bss_idx = i;
		}

		if (!strcmp(str, ".data")) {
			header->data_offset = RFLAT_SET_SEG_INDEX(section->sh_addr, i);
			header->data_size = section->sh_size;
			data_offset = section->sh_offset;
			data_size = ALIGN(section->sh_size, 4);
			data_idx = i;
		}

		if (!strcmp(str, ".got")) {
			header->got_size = section->sh_size;
		}

		if (!strcmp(str, ".symtab"))
			symtab = section;
		if (!strcmp(str, ".strtab"))
			strtab = section;
	}

	return 0;
}

int main(int argc, char **argv)
{
	int fd;
	FILE *fd_bin;
	Elf32_Shdr *shdr;
	struct stat sb;
	int ret = 0;
	struct rflat_header header;

	if (argc < 3)
		return -EINVAL;

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		printf("[-] Cannot open %s\n", argv[1]);
		return fd;
	}

	fd_bin = fopen(argv[2], "wb");
	if (!fd_bin) {
		printf("[-] Cannot open %s\n", argv[2]);
		return -errno;
	}

	fstat(fd, &sb);
	size = sb.st_size;
	buff = (char *)mmap((caddr_t)NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (buff < 0) {
		printf("[-] Cannot mmap file\n");
		close(fd);
		return errno;
	}

	debug_printf("[+] %s mapped at %p (size: %d)\n", argv[1], buff, size);

	ehdr = (Elf32_Ehdr *)buff;

	section_size = ehdr->e_shentsize;
	shdr = elf_get_section(ehdr->e_shstrndx);

	elf_parse(shdr, &header, buff);

	ret = elf_extract_reloc_infos(shdr, &header);
	if (ret < 0) {
		printf("[-] Failed to extract reloc infos\n");
		goto out;
	}

	header.magic = RFLAT_MAGIC;
	header.reloc_status = 0;
	header.text_offset = RFLAT_SET_SEG_INDEX((sizeof(header) + sizeof(struct rflat_reloc_infos) * header.reloc_n), text_idx);
	header.rodata_offset = RFLAT_SET_SEG_INDEX((RFLAT_GET_SEG_ADDR(header.text_offset) + text_size), rodata_idx);
	header.data_offset = RFLAT_SET_SEG_INDEX((RFLAT_GET_SEG_ADDR(header.text_offset) + text_size + rodata_size), data_idx);
	header.bss_offset = RFLAT_SET_SEG_INDEX(0, bss_idx);
	header.entry_point = RFLAT_GET_SEG_ADDR(header.text_offset) + ehdr->e_entry;
	header.reloc_infos = sizeof(header);

	debug_printf("entry_point: %x\n", header.entry_point);
	debug_printf("text_offset: %x\n", header.text_offset);
	debug_printf("rodata_offset: %x\n", header.rodata_offset);
	debug_printf("bss_offset: %x\n", header.bss_offset);
	debug_printf("data_offset: %x\n", header.data_offset);


	fwrite(&header, sizeof(header), 1, fd_bin);
	fwrite(reltab, sizeof(struct rflat_reloc_infos), header.reloc_n, fd_bin);
	fwrite(buff + text_offset, 1, text_size, fd_bin);
	fwrite(buff + rodata_offset, 1, rodata_size, fd_bin);
	fwrite(buff + data_offset, 1, data_size, fd_bin);

	free(reltab);

out:
	munmap((caddr_t)buff, size);
	close(fd);
	fclose(fd_bin);

	return ret;
}

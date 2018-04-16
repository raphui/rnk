/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <printk.h>
#include <string.h>
#include <errno.h>
#include <elf.h>
#include <mm.h>
#include <symbols.h>
#include <syscall.h>

/*
 * S (when used on its own) is the address of the symbol.
 * A is the addend for the relocation.
 * P is the address of the place being relocated (derived from r_offset).
 * T is 1 if the target symbol S has type STT_FUNC and the symbol addresses a Thumb instruction; it is 0
 * otherwise.
 */

#define R_ARM_NONE		0	/* Nothing */
#define R_ARM_ABS32		2	/* (S + A) | T */
#define R_ARM_THM_CALL		10	/* ((S + A) | T) – P */
#define R_ARM_CALL		28	/* ((S + A) | T) – P */
#define R_ARM_JUMP24		29	/* ((S + A) | T) – P */
#define R_ARM_V4BX		40	/* Nothing */
#define R_ARM_PREL31		42	/* (S + A) | T */
#define R_ARM_THM_MOVW_ABS_NC	47	/* (S + A) | T */
#define R_ARM_THM_MOVT_ABS	48	/* S + A */

#define ELF_APP_STACK_SIZE	2048

static char *buff;
static unsigned int ram_addr;
static int size;
static int section_size;
static elf32_ehdr *ehdr;
static elf32_shdr *symtab;
static elf32_shdr *strtab;
static unsigned int *lookup_table;
static int text_section_idx;

static elf32_shdr *elf_get_section(int num)
{
	return (elf32_shdr *)(buff + ehdr->e_shoff + num * section_size);
}

static int elf_get_symval(elf32_sym *sym)
{
	char *str;
	int addr = -ENXIO;
	elf32_shdr *target;

	if (ELF32_ST_BIND(sym->st_info) & (STB_GLOBAL | STB_WEAK)) {
		str = buff + strtab->sh_offset + sym->st_name;

		debug_printk("sym_value: %#x ", sym->st_value);
		debug_printk("sym: %s\n", str);

		addr = symbol_get_addr(str);
		if (addr < 0) {
			if (ELF32_ST_BIND(sym->st_info) & STB_WEAK)
				addr = 0;
			else if (sym->st_shndx != SHN_UNDEF) {
				addr = lookup_table[sym->st_shndx] + sym->st_value;

				debug_printk("sym: %s defined at 0x%x\n", str, addr);
			} else {
				error_printk("[-] Undefined symbol: %s\n", str);
				addr = -ENXIO;
			}
		}
	} else if (ELF32_ST_BIND(sym->st_info) & STB_LOCAL) {
		str = buff + strtab->sh_offset + sym->st_name;

		debug_printk("local sym: %s\n", str);

		target = elf_get_section(sym->st_shndx);
		addr = (int)(buff + sym->st_value + target->sh_offset);

		debug_printk("defined at 0x%x\n", addr);
	} else {
		addr = lookup_table[sym->st_shndx] + sym->st_value;
	}

	return addr;
}

static int elf_reloc(elf32_ehdr *ehdr, elf32_shdr *target, elf32_rel *rel)
{
	int addr = (int)(ram_addr);// + target->sh_offset);
	int *ref = (int *)(addr + rel->r_offset);
	elf32_sym *sym;
	int func;
	int ret = 0;
	int s, a, p, t;

	if (ELF32_R_SYM(rel->r_info) != SHN_UNDEF) {
		sym = (elf32_sym *)(buff + symtab->sh_offset + ELF32_R_SYM(rel->r_info) * symtab->sh_entsize);
		func = elf_get_symval(sym);
		if (func < 0) {
			func = 0;
		}


		debug_printk("\t- target: 0x%x\n", addr);
		debug_printk("\t- reloff in target: 0x%x\n", ref);
		debug_printk("\t- func: 0x%x\n", func);
	}

	s = func;
	a = *ref;
	p = (int)ref;
	t = func & 0x1;

	debug_printk("\t- %s instruction\n", t ? "Thumb" : "ARM");
	debug_printk("\t- before reloc: 0x%#x\n", *ref);

	switch (ELF32_R_TYPE(rel->r_info)) {
	case R_ARM_ABS32:
	case R_ARM_PREL31:
	case R_ARM_THM_MOVW_ABS_NC:
		*ref = (s + a) | t;
		break;
	case R_ARM_THM_CALL:
	case R_ARM_CALL:
	case R_ARM_JUMP24:
		*ref = ((s + a) | t) - p;
		break;
	case R_ARM_THM_MOVT_ABS:
		*ref = s + a;
		break;
	default:
		debug_printk("unknown reloc type\n");
		return -EINVAL;
	}

	debug_printk("\t- after reloc: 0x%x\n", *ref);

	return ret;
}

static int elf_section_alloc(elf32_shdr *shdr)
{
	int i;
	int ret = 0;
	char *str;
	void *mem;
	elf32_shdr *section;

	for (i = 0; i < ehdr->e_shnum; i++) {
		section = elf_get_section(i);
		str = buff + shdr->sh_offset + section->sh_name;

		if (!strcmp(str, ".text"))
			text_section_idx = i;

		if (section->sh_type & (SHT_NOBITS | SHT_PROGBITS)) {
			debug_printk("[!] %s\n", str);

			if (!section->sh_size) {
				debug_printk("[-] Section empty\n");
				continue;
			}

			if (section->sh_flags & SHF_ALLOC) {
				umalloc(section->sh_size, &mem);
				if (!mem) {
					error_printk("failed to alloc %d bytes for %s\n", section->sh_size, str);
					return -ENOMEM;
				}
				memset(mem, 0, section->sh_size);

				printk("Allocate %d bytes for section %s\n", section->sh_size, str);
				printk("Copying 0x%x bytes from 0x%x to 0x%x\n", section->sh_size, buff + section->sh_offset, mem);

				memcpy(mem, buff + section->sh_offset, section->sh_size);

				lookup_table[i] = (unsigned int)mem;
			}
		}
	}

	return ret;
}

static int elf_section_reloc(elf32_shdr *shdr)
{
	int i, j;
	int ret = 0;
	char *str;
	elf32_shdr *section;
	elf32_shdr *target;
	elf32_rel *rel;

	debug_printk("[+] dumping section needed relocation: \n");
	for (i = 0; i < ehdr->e_shnum; i++) {
		section = elf_get_section(i);
		str = buff + shdr->sh_offset + section->sh_name;
		if (section->sh_type == SHT_REL) {
			debug_printk("[!] %s\n", str);
			for (j = 0; j < (section->sh_size / section->sh_entsize); j++) {
				rel = (elf32_rel *)(buff + section->sh_offset + j * section->sh_entsize);
				target = elf_get_section(section->sh_info);

				debug_printk("\t- offset: 0x%x ", rel->r_offset);
				debug_printk("info: 0x%x \n", rel->r_info);
				debug_printk("\t- applies to: 0x%x\n", section->sh_info);

				ram_addr = lookup_table[section->sh_info];
				ret = elf_reloc(ehdr, target, rel);
				if (ret < 0) {
					debug_printk("[-] Failed to reloc\n");
				}
			}
		}
	}

	return ret;
}

static int elf_check_type(elf32_ehdr *ehdr)
{
	int ret = 0;

	if (!ehdr)
		return -ENOENT;

	if (ehdr->e_ident[EI_MAG0] != ELFMAG0) {
		ret = -EINVAL;
		goto out;
	}

	if (ehdr->e_ident[EI_MAG1] != ELFMAG1) {
		ret = -EINVAL;
		goto out;
	}

	if (ehdr->e_ident[EI_MAG2] != ELFMAG2) {
		ret = -EINVAL;
		goto out;
	}

	if (ehdr->e_ident[EI_MAG3] != ELFMAG3) {
		ret = -EINVAL;
		goto out;
	}

out:
	return ret;
}

int elf_load(char *elf_data, int elf_size, int reloc_addr)
{
	char *str;
	elf32_shdr *shdr;
	elf32_shdr *section;
	int i;
	int ret = 0;

	size = elf_size;
	buff = elf_data;

	printk("[+] ELF at %p (size: %d)\n", buff, size);

	ehdr = (elf32_ehdr *)buff;

	ret = elf_check_type(ehdr);
	if (ret < 0) {
		error_printk("Invalid ELF format\n");
		goto out;
	}

	debug_printk("[+] Info: \n");
	debug_printk("\t- bit format: %#x\n", ehdr->e_ident[EI_CLASS]);
	debug_printk("\t- architecture: %#x\n", ehdr->e_machine);
	debug_printk("\t- num section: %d\n", ehdr->e_shnum);
	debug_printk("\t- index sections name: %d\n", ehdr->e_shstrndx);
	debug_printk("\t- section header offset: %#x\n", ehdr->e_shoff);

	lookup_table = (unsigned int *)kmalloc(ehdr->e_shnum * sizeof(unsigned int));
	memset(lookup_table, 0, ehdr->e_shnum);

	section_size = ehdr->e_shentsize;
	shdr = elf_get_section(ehdr->e_shstrndx);

	debug_printk("\t- section string table offset: %#x\n", shdr->sh_offset);

	debug_printk("[+] dumping section name: \n");
	for (i = 0; i < ehdr->e_shnum; i++) {
		section = elf_get_section(i);
		str = buff + shdr->sh_offset + section->sh_name;
		debug_printk("[!] %s\n", str);

		if (!strcmp(str, ".symtab"))
			symtab = section;
		else if (!strcmp(str, ".strtab"))
			strtab = section;
	}

	ret = elf_section_alloc(shdr);
	if (ret < 0) {
		printk("[-] Failed to allocate sections\n");
		goto out;
	}

	ret = elf_section_reloc(shdr);
	if (ret < 0)
		printk("[-] Failed to reloc sections\n");

out:
	return ret;
}

static void elf_jump(unsigned int entry)
{
	unsigned int (*fct)(void) = (void *)entry;
	fct();
}

int elf_exec(char *elf_data, int elf_size, int reloc_addr)
{
	int ret = 0;
	unsigned int entry_point;

	ret = elf_load(elf_data, elf_size, reloc_addr);
	if (ret < 0) {
		error_printk("failed to load elf\n");
		return -EIO;
	}

	entry_point = lookup_table[text_section_idx] + ehdr->e_entry;
	kfree(lookup_table);
	debug_printk("ph_off: 0x%x\r\n", ehdr->e_phoff);
	debug_printk("elf entry point at: 0x%x\r\n", entry_point);

	ret = entry_point;

	return ret;
}

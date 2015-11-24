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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <elf.h>

#include "symbols.h"

#define ELF_ST_BIND(x)	((x) >> 4)

/*
 * S (when used on its own) is the address of the symbol.
 * A is the addend for the relocation.
 * P is the address of the place being relocated (derived from r_offset).
 * T is 1 if the target symbol S has type STT_FUNC and the symbol addresses a Thumb instruction; it is 0
 * otherwise.
 */

#define R_ARM_ABS32		2	/* (S + A) | T */
#define R_ARM_THM_CALL		10	/* ((S + A) | T) – P */
#define R_ARM_THM_MOVW_ABS_NC	47	/* (S + A) | T */
#define R_ARM_THM_MOVT_ABS	48	/* S + A */

static char *buff;
static int size;
static int section_size;
static elf32_ehdr *ehdr;
static elf32_shdr *symtab;
static elf32_shdr *strtab;

static elf32_shdr *elf_get_section(int num)
{
	return (elf32_shdr *)(buff + ehdr->e_shoff + num * section_size);
}

static int elf_get_symval(elf32_sym *sym)
{
	char *str;
	int addr;
	elf32_shdr *target;

	if (ELF32_ST_BIND(sym->st_info) & (STB_GLOBAL | STB_WEAK)) {
		str = buff + strtab->sh_offset + sym->st_name;

		debug_printk("sym_value: %#x ", sym->st_value);
		debug_printk("sym: %s\n", str);

		addr = symbol_get_addr(str);
		if (addr == 0) {
			if (ELF32_ST_BIND(sym->st_info) & STB_WEAK)
				addr = 0;
			else {
				printk("[-] Undefined symbol: %s\n", str);
				addr = -ENXIO;
			}
		}
	}

	return addr;
}

static int elf_reloc(elf32_ehdr *ehdr, elf32_shdr *target, elf32_rel *rel)
{
	int addr = buff + target->sh_offset;
	int *ref = (int *)(addr + rel->r_offset);
	elf32_sym *sym;
	int func;
	int ret = 0;
	int s, a, p, t;

	if (ELF32_R_SYM(rel->r_info) != SHN_UNDEF) {
		sym = (elf32_sym *)(buff + symtab->sh_offset + ELF32_R_SYM(rel->r_info) * symtab->sh_entsize);
		func = elf_get_symval(sym);
		if (func < 0)
			return func;

		debug_printk("\t- target: %#x\n", addr);
		debug_printk("\t- reloff in target: %#p\n", ref);
		debug_printk("\t- func: %#x\n", func);

		if (!func) {
			debug_printk("[-] Failed to find address symbol\n");
			return -ENXIO;
		}
	}

	s = func;
	a = *ref;
	p = ref;
	t = func & 0x1;

	debug_printk("\t- %s instruction\n", t ? "Thumb" : "ARM");
	debug_printk("\t- before reloc: %#x\n", *ref);

	switch (ELF32_R_TYPE(rel->r_info)) {
	case R_ARM_ABS32:
		debug_printk("R_ARM_ABS32 reloc\n");
		*ref = (s + a) | t;
		break;
	case R_ARM_THM_CALL:
		debug_printk("R_ARM_THM_CALL reloc\n");
		*ref = ((s + a) | t) - p;
		break;
	case R_ARM_THM_MOVW_ABS_NC:
		debug_printk("R_ARM_THM_MOVW_ABS_NC reloc\n");
		*ref = (s + a) | t;
		break;
	case R_ARM_THM_MOVT_ABS:
		debug_printk("R_ARM_THM_MOVT_ABS reloc\n");
		*ref = s + a;
		break;
	default:
		return -EINVAL;
	}

	debug_printk("\t- after reloc: %#x\n", *ref);

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
		if (section->sh_type == SHT_NOBITS) {
			printk("[!] %s\n", str);

			if (!section->sh_size) {
				printk("[-] Section empty\n");
				continue;
			}

			if (section->sh_flags & SHF_ALLOC) {
				mem = kmalloc(section->sh_size);
				memset(mem, 0, section->sh_size);

				section->sh_offset = (int)mem - (int)buff;
				printk("Allocate %d bytes for section %s\n", section->sh_size, str);
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

	printk("[+] dumping section needed relocation: \n");
	for (i = 0; i < ehdr->e_shnum; i++) {
		section = elf_get_section(i);
		str = buff + shdr->sh_offset + section->sh_name;
		if (section->sh_type == SHT_REL) {
			printk("[!] %s\n", str);
			for (j = 0; j < (section->sh_size / section->sh_entsize); j++) {
				rel = (elf32_rel *)(buff + section->sh_offset + j * section->sh_entsize);
				target = elf_get_section(section->sh_info);

				debug_printk("\t- offset: %#x ", rel->r_offset);
				debug_printk("info: %#x ", rel->r_info);
				debug_printk("\t- applies to: %#x\n", section->sh_info);

				ret = elf_reloc(ehdr, target, rel);
				if (ret < 0) {
					debug_printk("[-] Failed to reloc\n");
				}
			}
		}
	}

	return ret;
}

int elf_load(char *elf_data, int elf_size, int reloc_addr)
{
	int fd;
	char *str;
	elf32_shdr *shdr;
	elf32_shdr *section;
	int i;
	int ret = 0;

	size = elf_size;
	buff = elf_data;

	printk("[+] ELF at %p (size: %d)\n", buff, size);

	ehdr = (elf32_ehdr *)buff;

	printk("[+] Info: \n");
	printk("\t- bit format: %#x\n", ehdr->e_ident[EI_CLASS]);
	printk("\t- architecture: %#x\n", ehdr->e_machine);
	printk("\t- num section: %d\n", ehdr->e_shnum);
	printk("\t- index sections name: %d\n", ehdr->e_shstrndx);
	printk("\t- section header offset: %#x\n", ehdr->e_shoff);

	section_size = ehdr->e_shentsize;
	shdr = elf_get_section(ehdr->e_shstrndx);

	printk("\t- section string table offset: %#x\n", shdr->sh_offset);

	printk("[+] dumping section name: \n");
	for (i = 0; i < ehdr->e_shnum; i++) {
		section = elf_get_section(i);
		str = buff + shdr->sh_offset + section->sh_name;
		printk("[!] %s\n", str);

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

	ehdr->e_entry = reloc_addr;

out:
	return ret;
}

#ifndef ELFLOADER_H
#define ELFLOADER_H

int elf_load(char *elf_data, int elf_size, int reloc_addr);
int elf_exec(char *elf_data, int elf_size, int reloc_addr);

#endif /* ELFLOADER_H */

#ifndef SYMBOLS_H
#define SYMBOLS_H

struct symbol {
	unsigned int addr;
	char *name;
};

int symbol_get_addr_by_hash(unsigned int hash);
char *symbol_get_name(unsigned int addr);
int symbol_get_addr(char *name);

#endif /* SYMBOLS_H */

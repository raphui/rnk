#ifndef SYMBOLS_H
#define SYMBOLS_H

struct symbol {
	unsigned int addr;
	char *name;
};

extern char *symbol_get_name(unsigned int addr);
extern int symbol_get_addr(char *name);

#endif /* SYMBOLS_H */

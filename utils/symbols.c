#include <stdarg.h>
#include <kernel/printk.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <symbols.h>

extern struct symbol __rnk_ksym_start[];
extern struct symbol __rnk_ksym_end[];

static unsigned int hash_str(char *str)
{
	unsigned int hash = 5381;
	int c;

	while ((c = *str++))
		hash = ((hash << 5) + hash) + c; /*  hash * 33 + c */

	return hash;
}

int symbol_get_addr_by_hash(unsigned int hash)
{
	int ret = -ENXIO;
	struct symbol *sym;

	for (sym = __rnk_ksym_start; sym < __rnk_ksym_end; sym++) {
		if (hash_str(sym->name) == hash) {
			ret = sym->addr;
			break;
		}
	}

	return ret;

}

char *symbol_get_name(unsigned int addr)
{
	char *ret = NULL;
	struct symbol *sym;

	for (sym = __rnk_ksym_start; sym < __rnk_ksym_end; sym++) {
		if (sym->addr == addr) {
			ret = sym->name;
			break;
		}
	}

	return ret;
}

int symbol_get_addr(char *name)
{
	int ret = -ENXIO;
	struct symbol *sym;

	for (sym = __rnk_ksym_start; sym < __rnk_ksym_end; sym++) {
		if (!strcmp(name, sym->name)) {
			ret = sym->addr;
			break;
		}
	}

	return ret;
}

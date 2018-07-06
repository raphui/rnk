#include <stdarg.h>
#include <kernel/printk.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <symbols.h>

extern struct symbol __rnk_ksym_start[];
extern struct symbol __rnk_ksym_end[];

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

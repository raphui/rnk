#ifndef EXPORT_H
#define EXPORT_H

#include <symbols.h>

#define EXPORT_SYMBOL(sym)       static const struct symbol __ksym_##sym __attribute__((__used__)) \
	__attribute__((__section__(".ksym"))) = {(unsigned int)&sym, #sym}

#endif /* EXPORT_H */

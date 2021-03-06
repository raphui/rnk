#ifndef STDLIB_H
#define STDLIB_H

#include <stdio.h>
#include <stddef.h>

#ifdef CONFIG_DEBUG
#define assert(x) do { if (!(x)) { printf("ASSERT FAILED at (%s:%d): %s\n", __FILE__, __LINE__, #x); for (;;){}; } } while(0)
#else
#define assert(x)
#endif

void *malloc(size_t size);
void free(void *mem);

#endif /* STDLIB_H */

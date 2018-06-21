#ifndef STDLIB_H
#define STDLIB_H

#include <stddef.h>

#define assert(x) if (!(x)) { for (;;){} }

void *malloc(size_t size);
void free(void *mem);

#endif /* STDLIB_H */

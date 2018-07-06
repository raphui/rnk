#include <stdlib.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <export.h>

void *malloc(size_t size)
{
	unsigned int mem;

	syscall(SYSCALL_ALLOC, size, &mem);

	return (void *)mem;
}
EXPORT_SYMBOL(malloc);


void free(void *mem)
{
	syscall(SYSCALL_FREE, mem);
}
EXPORT_SYMBOL(free);

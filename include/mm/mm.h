#ifndef MM_H
#define MM_H

#include <stddef.h>

extern int heap_size;
extern int mem_alloc;
#define mem_available (heap_size - mem_alloc)

extern int heap_init(void);
extern void *kmalloc(size_t size);
extern void kfree(void *mem);
extern void umalloc(size_t size, void *m);
extern void ufree(void *mem);

#ifdef CONFIG_DLMALLOC
extern void *sbrk(int increment);
#endif /* CONFIG_DLMALLOC */


#endif /* MM_H */

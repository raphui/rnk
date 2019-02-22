#ifndef PRINTK_H
#define PRINTK_H

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#define SEEK_SET	0
#define SEEK_CUR	1
#define SEEK_END	2

#ifdef CONFIG_VERBOSE
#define verbose_printk(...) do{ printk(__VA_ARGS__); }while(0)
#else
#define verbose_printk(...)
#endif

#ifdef CONFIG_DEBUG
#define debug_printk(...) do{ printk(__VA_ARGS__); }while(0)
#define assert(x) do { if (!(x)) { printf("ASSERT FAILED at (%s:%d): %s\n", __FILE__, __LINE__, #x); for (;;){}; } } while(0)
#else
#define debug_printk(...)
#define assert(x)
#endif

#ifdef CONFIG_ERROR
#define error_printk(...) do{ printk(__VA_ARGS__); }while(0)
#else
#define error_printk(...)
#endif


void printk(char *fmt, ...);

#endif /* PRINTK_H */

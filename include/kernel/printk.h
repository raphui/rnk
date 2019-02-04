#ifndef PRINTK_H
#define PRINTK_H

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#define SEEK_SET	0
#define SEEK_CUR	1
#define SEEK_END	2

#ifdef CONFIG_DEBUG
#define DEBUG	1
#else
#define DEBUG	0
#endif

#ifdef CONFIG_ERROR
#define ERROR	1
#else
#define ERROR	0
#endif

#ifdef CONFIG_VERBOSE
#define VERBOSE 1
#else
#define VERBOSE 0
#endif

#define verbose_printk(...) do{ if(VERBOSE){ printk(__VA_ARGS__); } }while(0)
#define debug_printk(...) do{ if(DEBUG){ printk(__VA_ARGS__); } }while(0)
#define error_printk(...) do{ if(ERROR){ printk(__VA_ARGS__); } }while(0)
#define assert(x) do { if (!(x)) { printf("ASSERT FAILED at (%s:%d): %s\n", __FILE__, __LINE__, #x); for (;;){}; } } while(0)

void printk(char *fmt, ...);

#endif /* PRINTK_H */

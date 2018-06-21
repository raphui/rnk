#ifndef STDIO_H
#define STDIO_H

#include <stdarg.h>
#include <stddef.h>

#define FILE	long

#define SEEK_SET	0
#define SEEK_CUR	1
#define SEEK_END	2

void printf(char *fmt, ...);
void vprintf(char *fmt, va_list va);

FILE *fopen(const char *path, const char *mode);
int fclose(FILE *stream); 
size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);

#endif /* STDIO_H */

#ifndef STRING_H
#define STRING_H

#include <stddef.h>

void memcpy(void *dst, const void *src, unsigned int num);
void *memset(void *s, int c, unsigned int count);
int strcmp(const char *cs, const char *ct);
void *memchr(const void *ptr, int value, size_t num);
int memcmp(const void *ptr1, const void *ptr2, size_t num);
void memmove(void *dst, const void *src, size_t n);
size_t strlen(const char *s);
char *strcpy(char *dst, const char *src);

#endif /* STRING_H */

#include <string.h>
#include <export.h>

void memcpy(void *dst, const void *src, unsigned int num)
{
	const unsigned char *s = src;
	unsigned char *d = dst;

	while (num--) {
		*d++ = *s++;
	}
}
EXPORT_SYMBOL(memcpy);

void *memset(void *s, int c, unsigned int count)
{
	char *xs = (char *)s;

	while (count--)
		*xs++ = c;

	return s;
}
EXPORT_SYMBOL(memset);

int strcmp(const char *cs, const char *ct)
{
	char res;

	while (1) {
		if ((res = *cs - *ct++) != 0 || !*cs++)
			break;
	}

	return res;
}
EXPORT_SYMBOL(strcmp);

void *memchr(const void *ptr, int value, size_t num)
{
    const unsigned char *p = ptr;

    while (num--) {
        if (*p == value) {
            return (void *) p;
        }
        p++;
    }

    return NULL;
}
EXPORT_SYMBOL(memchr);

int memcmp(const void *ptr1, const void *ptr2, size_t num)
{
    const unsigned char *p1 = ptr1;
    const unsigned char *p2 = ptr2;

    while (num--) {
        if (*p1 != *p2) {
            if (*p1 > *p2) {
                return 1;
            }
            else {
                return -1;
            }
        }

        p1++;
        p2++;
    }

    return 0;
}
EXPORT_SYMBOL(memcmp);

void memmove(void *dst, const void *src, size_t n)
{
    const char *s = src;
    char *d = dst;
    if ((unsigned int *)s < (unsigned int *)d)
        while(n--) d[n] = s[n];
    else
        while(n--) *d++ = *s++;
}
EXPORT_SYMBOL(memmove);

size_t strlen(const char *s)
{
    size_t len = 0;
    while (*s++) {
        len++;
    }
    return len;
}
EXPORT_SYMBOL(strlen);

char *strcpy(char *dst, const char *src)
{
	while (*src++)
		*dst++ = *src;

	return dst;
}
EXPORT_SYMBOL(strcpy);

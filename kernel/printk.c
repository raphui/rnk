#include <printk.h>
#include <errno.h>
#include <console.h>

#include <printf.h>

void k_putchar(char c)
{
	char r = '\r';

	if (c == '\n')
		console_write((unsigned char *)&r, sizeof(unsigned char));

	console_write((unsigned char *)&c, sizeof(unsigned char));
}

void printk(char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);

	__printk(fmt, va);
}

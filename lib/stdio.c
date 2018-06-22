#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <export.h>

#ifdef CONFIG_SEMIHOSTING
#include <arch/semihosting.h>
#endif /* CONFIG_SEMIHOSTING */

/* XXX: armv7m mpu need a least 32 bytes to map a section,
 * this ensure that .bss will be map for user access
 **/
static char dummy[32];

FILE *fopen(const char *path, const char *mode)
{
#ifdef CONFIG_SEMIHOSTING

	return smh_open(path, (char *)mode);
#else
	return (FILE *)ERR_PTR(-ENOSYS);

#endif /* CONFIG_SEMIHOSTING */
}

int fclose(FILE *stream)
{
#ifdef CONFIG_SEMIHOSTING

	return smh_close((long)stream);
#else
	return -ENOSYS;

#endif /* CONFIG_SEMIHOSTING */
}

size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream)
{
#ifdef CONFIG_SEMIHOSTING

	return smh_write((long)stream, (void *)ptr, size * nmemb);
#else
	return -ENOSYS;

#endif /* CONFIG_SEMIHOSTING */
}

size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
#ifdef CONFIG_SEMIHOSTING

	return smh_read((long)stream, (void *)ptr, size * nmemb);
#else
	return -ENOSYS;

#endif /* CONFIG_SEMIHOSTING */
}

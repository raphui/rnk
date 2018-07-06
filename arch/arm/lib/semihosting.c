#include <stddef.h>
#include <string.h>

#include <drv/console.h>

#define SYSOPEN		0x01
#define SYSCLOSE	0x02
#define SYSWRITEC	0x03
#define SYSWRITE0	0x04
#define SYSWRITE	0x05
#define SYSREAD		0x06
#define SYSFLEN		0x0C

#define MODE_READ	0x0
#define MODE_READBIN	0x1

#define MODE_WRITE	0x4
#define MODE_WRITEBIN	0x5

#ifdef CONFIG_SEMIHOSTING_DEBUG_BUFFERED

#define PRINT_BUFFER_SIZE	128

static char print_buffer[PRINT_BUFFER_SIZE];
static int print_n;

#endif /* CONFIG_SEMIHOSTING_DEBUG_BUFFERED */

/*
 * Call the handler
 */
static long smh_trap(unsigned int sysnum, void *addr)
{
	register long result asm("r0");
#if defined(CONFIG_ARM64)
	asm volatile ("hlt #0xf000" : "=r" (result) : "0"(sysnum), "r"(addr));
#elif defined(CONFIG_CPU_ARMV7M)
	asm volatile ("bkpt #0xAB" : "=r" (result) : "0"(sysnum), "r"(addr));
#else
	/* Note - untested placeholder */
	asm volatile ("svc #0x123456" : "=r" (result) : "0"(sysnum), "r"(addr));
#endif
	return result;
}

/*
 * Open a file on the host. Mode is "r" or "rb" currently. Returns a file
 * descriptor or -1 on error.
 */
long smh_open(const char *fname, char *modestr)
{
	long fd;
	unsigned long mode;
	struct smh_open_s {
		const char *fname;
		unsigned long mode;
		size_t len;
	} open;

	debug_printk("%s: file \'%s\', mode \'%s\'\n", __func__, fname, modestr);

	/* Check the file mode */
	if (!(strcmp(modestr, "r"))) {
		mode = MODE_READ;
	} else if (!(strcmp(modestr, "rb"))) {
		mode = MODE_READBIN;
	} else if (!(strcmp(modestr, "w"))) {
		mode = MODE_WRITE;
	} else if (!(strcmp(modestr, "wb"))) {
		mode = MODE_WRITEBIN;
	} else {
		error_printk("%s: ERROR mode \'%s\' not supported\n", __func__,
		       modestr);
		return -1;
	}

	open.fname = fname;
	open.len = strlen(fname);
	open.mode = mode;

	/* Open the file on the host */
	fd = smh_trap(SYSOPEN, &open);
	if (fd == -1)
		error_printk("%s: ERROR fd %ld for file \'%s\'\n", __func__, fd,
		       fname);

	return fd;
}

static int smh_write0(struct device *dev, unsigned char *buff, unsigned int len)
{
	smh_trap(SYSWRITE0, buff);

	return 0;
}

static int smh_writec(struct device *dev, unsigned char *buff, unsigned int len)
{
	int i;

#ifdef CONFIG_SEMIHOSTING_DEBUG_BUFFERED
	for (i = 0; i < len; i++) {
		if (print_n == (PRINT_BUFFER_SIZE - 1)) {
			print_buffer[print_n] = '\0';

			smh_write0(dev, print_buffer, PRINT_BUFFER_SIZE);
	
			print_n = 0;
			print_buffer[print_n] = buff[i];
		} else {
			print_buffer[print_n] = buff[i];
		}

		print_n++;
	}
#else
	struct smh_writec_s {
		char memp;
	} writec;

	for (i = 0; i < len; i++) {

		writec.memp = buff[i];

		smh_trap(SYSWRITEC, &writec);
	}
#endif

	return 0;
}

/*
 * Write 'len' bytes of mem into 'fd'. Returns 0 on success, else failure
 */
long smh_write(long fd, void *memp, size_t len)
{
	long ret;
	struct smh_write_s {
		long fd;
		void *memp;
		size_t len;
	} write;

	debug_printk("%s: fd %ld, memp %p, len %zu\n", __func__, fd, memp, len);

	write.fd = fd;
	write.memp = memp;
	write.len = len;

	ret = smh_trap(SYSWRITE, &write);
	if (ret < 0) {
		/*
		 * The ARM handler allows for returning partial lengths,
		 * but in practice this never happens so rather than create
		 * hard to maintain partial read loops and such, just fail
		 * with an error message.
		 */
		error_printk("%s: ERROR ret %ld, fd %ld, len %zu memp %p\n",
		       __func__, ret, fd, len, memp);
		return -1;
	}

	return 0;
}

/*
 * Read 'len' bytes of file into 'memp'. Returns 0 on success, else failure
 */
long smh_read(long fd, void *memp, size_t len)
{
	long ret;
	struct smh_read_s {
		long fd;
		void *memp;
		size_t len;
	} read;

	debug_printk("%s: fd %ld, memp %p, len %zu\n", __func__, fd, memp, len);

	read.fd = fd;
	read.memp = memp;
	read.len = len;

	ret = smh_trap(SYSREAD, &read);
	if (ret < 0) {
		/*
		 * The ARM handler allows for returning partial lengths,
		 * but in practice this never happens so rather than create
		 * hard to maintain partial read loops and such, just fail
		 * with an error message.
		 */
		error_printk("%s: ERROR ret %ld, fd %ld, len %zu memp %p\n",
		       __func__, ret, fd, len, memp);
		return -1;
	}

	return 0;
}

/*
 * Close the file using the file descriptor
 */
long smh_close(long fd)
{
	long ret;

	debug_printk("%s: fd %ld\n", __func__, fd);

	ret = smh_trap(SYSCLOSE, &fd);
	if (ret == -1)
		error_printk("%s: ERROR fd %ld\n", __func__, fd);

	return ret;
}

/*
 * Get the file length from the file descriptor
 */
static long smh_len_fd(long fd)
{
	long ret;

	debug_printk("%s: fd %ld\n", __func__, fd);

	ret = smh_trap(SYSFLEN, &fd);
	if (ret == -1)
		error_printk("%s: ERROR ret %ld, fd %ld\n", __func__, ret, fd);

	return ret;
}

static int smh_load_file(const char * const name, unsigned long load_addr,
			 unsigned long *end_addr)
{
	long fd;
	long len;
	long ret;

	fd = smh_open(name, "rb");
	if (fd == -1)
		return -1;

	len = smh_len_fd(fd);
	if (len < 0) {
		smh_close(fd);
		return -1;
	}

	ret = smh_read(fd, (void *)load_addr, len);
	smh_close(fd);

	if (ret == 0) {
		*end_addr = load_addr + len - 1;
		printk("loaded file %s from %08lX to %08lX, %08lX bytes\n",
		       name,
		       load_addr,
		       *end_addr,
		       len);
	} else {
		error_printk("read failed\n");
		return 0;
	}

	return 0;
}

#ifdef CONFIG_SEMIHOSTING_DEBUG
struct io_operations io_op = {
	.write = smh_writec,
};
#endif /* CONFIG_SEMIHOSTING_DEBUG */

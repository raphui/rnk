#ifndef IO_OPS_H
#define IO_OPS_H

#include <stddef.h>
#include <kernel/printk.h>

#define O_RDONLY        0
#define O_WRONLY        1
#define O_RDWR          2

int svc_open(const char *path, int flags);
int svc_close(int fd);
int svc_write(int fd, const void *buf, size_t size);
int svc_read(int fd, void *buf, size_t size);
int svc_lseek(int fd, int offset, int whence);

#endif /* IO_OPS_H */

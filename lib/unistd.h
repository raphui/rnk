#ifndef UNISTD_H
#define UNISTD_H

#include <stddef.h>

#define O_RDONLY        0
#define O_WRONLY        1
#define O_RDWR          2

#define O_CREAT		0x100
#define O_EXCL		0x200
#define O_APPEND	0x1000
#define O_NONBLOCK	0x2000

int open(const char *path, int flags);
int close(int fd);
int write(int fd, const void *buf, size_t size);
int read(int fd, void *buf, size_t size);
int lseek(int fd, int offset, int whence);

int syscall(int number, ...);

#endif /* UNISTD_H */

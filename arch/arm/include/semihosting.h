#ifndef SEMIHOSTING_H
#define SEMIHOSTING_H

long smh_open(const char *fname, char *modestr);
long smh_write(long fd, void *memp, size_t len);
long smh_read(long fd, void *memp, size_t len);
long smh_close(long fd);

#endif /* SEMIHOSTING_H */

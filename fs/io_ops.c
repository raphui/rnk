#include <errno.h>
#include <kernel/printk.h>
#include <drv/device.h>
#include <unistd.h>

#define MAX_FD	8

struct device_io {
	struct device *dev;
	int (*read)(struct device *dev, unsigned char *buff, unsigned int size);
	int (*write)(struct device *dev, const unsigned char *buff, unsigned int size);
	int (*lseek)(struct device *dev, int offset, int whence);
	int (*ioctl)(struct device *dev, int request, char *arg);
	int perm;
};

static struct device_io devs[MAX_FD];

static int fd_num = 0;

int svc_open(const char *path, int flags)
{
	int ret = fd_num;
	struct device *dev;

	if (fd_num < MAX_FD) {
		dev = device_from_name(path);
		if (dev) {

			if (dev->open) {
				ret = dev->open(dev);
				if (ret < 0)
					goto err;
			}

			devs[fd_num].dev = dev;
			devs[fd_num].read = dev->read;
			devs[fd_num].write = dev->write;
			devs[fd_num].lseek = dev->lseek;
			devs[fd_num].ioctl = dev->ioctl;
			devs[fd_num].perm = flags;

			fd_num++;

		} else {
			error_printk("invalid open path\n");
			ret = -ENOENT;
		}


	} else {
		error_printk("no more file descriptor left\n");
		ret = -EMFILE;
	}

err:
	return ret;
}

int svc_close(int fd)
{
	int ret = 0;

	if (fd < MAX_FD) {
		devs[fd].dev = NULL;
		devs[fd].read = NULL;
		devs[fd].write = NULL;
		devs[fd].lseek = NULL;
		devs[fd].perm = 0;

		fd_num--;
	} else {
		error_printk("invalid fd\n");
		ret = -EBADF;
	}

	return ret;
}

int svc_write(int fd, const void *buf, size_t size)
{
	int ret = 0;

	if (fd >= fd_num) {
		error_printk("invalid file descriptor\n");
		return -EBADF;
	}

	if (devs[fd].perm & (O_WRONLY | O_RDWR)) {
		if (devs[fd].write) {

			ret = devs[fd].write(devs[fd].dev, buf, size);

		} else {
			error_printk("cannot write to fd: %d\n", fd);
			ret = -ENOSYS;
		}
	} else {
		error_printk("forbidden writing to fd: %d\n", fd);
		ret = -EPERM;
	}


	return ret;
}

int svc_read(int fd, void *buf, size_t size)
{
	int ret = 0;

	if (fd >= fd_num) {
		error_printk("invalid file descriptor\n");
		return -EBADF;
	}

	if (devs[fd].perm & (O_RDONLY | O_RDWR)) {
		if (devs[fd].read) {

			ret = devs[fd].read(devs[fd].dev, buf, size);

		} else {
			error_printk("cannot read to fd: %d\n", fd);
			ret = -ENOSYS;
		}
	} else {
		error_printk("forbidden reading to fd: %d\n", fd);
		ret = -EPERM;
	}

	return ret;
}

int svc_lseek(int fd, int offset, int whence)
{
	int ret = 0;

	if (fd >= fd_num) {
		error_printk("invalid file descriptor\n");
		return -EBADF;
	}

	if ((whence != SEEK_SET) && (whence != SEEK_CUR) && (whence != SEEK_END)) {
		error_printk("invalid whence\n");
		return -EINVAL;
	}

	if (devs[fd].lseek)
		ret = devs[fd].lseek(devs[fd].dev, offset, whence);
	else {
		error_printk("cannot lseek on fd: %d\n", fd);
		ret = -ENOSYS;
	}

	return ret;
}

int svc_ioctl(int fd, int request, char *arg)
{
	int ret = 0;

	if (fd >= fd_num) {
		error_printk("invalid file descriptor\n");
		return -EBADF;
	}

	if (devs[fd].ioctl) {
		ret = devs[fd].ioctl(devs[fd].dev, request, arg);
	} else {
		error_printk("cannot ioctl to fd: %d\n", fd);
		ret = -ENOSYS;
	}

	return ret;
}

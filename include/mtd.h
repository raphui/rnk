#ifndef MTD_H
#define MTD_H

#include <device.h>
#include <list.h>

#define MAX_SECTORS	32

struct mtd;
struct mtd_page;

struct mtd_operations
{
	int (*erase)(struct mtd *mtd, unsigned int sector);
	int (*write)(struct mtd *mtd, unsigned char *buff, unsigned int size, struct mtd_page *page);
	int (*read)(struct mtd *mtd, unsigned char *buff, unsigned int size);
};

struct mtd_layout {
	unsigned int pages_count;
	unsigned int pages_size;
};

struct mtd {
	unsigned int base_addr;
	unsigned int total_size;
	int layout_size;
	int curr_off;
	struct mtd_layout *mtd_map;
	struct mtd_operations *mtd_ops;
	struct device dev;
	struct list_node node;
};

struct mtd_page {
	unsigned int start;
	unsigned int end;
	unsigned int index;
};

struct mtd *mtd_new_controller(void);
int mtd_remove_controller(struct mtd *mtd);
int mtd_register_controller(struct mtd *mtd);
int mtd_init(void);

#endif /* MTD_H */

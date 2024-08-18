#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "main.h"
#include "usb_com.h"
#include "update.h"

#define USB_PACKET	4192
#define DEFAULT_COM_PORT "/dev/ttyACM0\0"

struct option long_options[] = {
	{"port", required_argument, NULL, 'p'},
	{"update", required_argument, NULL, 'u'},
	{ NULL, 0, NULL, 0},
};

static void usage(char *program_name)
{
	fprintf(stderr, "Usage: %s [--port com_port] command\n", program_name);
}

int main(int argc, char **argv)
{
	int opt, fw_fd, fw_size;
	int update = 0;
	int ret = 0;
	int upd_type;
	int use_default_com_port = 1;
	char com_port[32];
	char fw_path[1024];
	char *fw;
	struct stat sb;

	if (!argv[optind]) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	memset(com_port, 0, sizeof(com_port));

	while ((opt = getopt_long(argc, argv, "c:p:r:", long_options, NULL)) != -1) {
		switch (opt) {
			case 'p': 
				strncpy(com_port, optarg, strlen(optarg));
				use_default_com_port = 0;
			break;
			case 'u':
				strncpy(fw_path, optarg, strlen(optarg));

				printf("%s\n", fw_path);
				if (strstr(fw_path, "lr1110")) {
					upd_type = UPD_LORA_FW;	
				}

				update = 1;
			break;
			default:
				usage(argv[0]);
				exit(EXIT_FAILURE);
		}
	}

	if (use_default_com_port) {
		memcpy(com_port, DEFAULT_COM_PORT, strlen(DEFAULT_COM_PORT));
	}

	printf("use com_port: %s\n", com_port);

	ret = usb_open(com_port);

	if (update) {
		fw_fd = open(fw_path, O_RDWR);
		if (fw_fd < 0) {
			printf("fail to open %s\n", fw_path);
			ret = -EIO;
			goto err;
		}

		fstat(fw_fd, &sb);
		fw_size = sb.st_size;

		fw = (char *)mmap((caddr_t)NULL, fw_size, PROT_READ, MAP_SHARED, fw_fd, 0);
		if (fw < 0) {
			printf("fail to mmap file %s\n", fw_path);
			ret = -EIO;
			close(fw_fd);
			goto err;
		}

		printf("update...");
		ret = update_fw(upd_type, fw, fw_size);
		printf("%x\n", ret);

		close(fw_fd);
		munmap(fw, fw_size);
	}

err:
	return ret;
}

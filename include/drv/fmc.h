#ifndef FMC_H
#define FMC_H

#include <drv/device.h>

struct fmc_sdram_cmd_config {
	unsigned int cmd_mode;
	unsigned int cmd_target;
	unsigned int auto_refresh_num;
	unsigned int mode;
};

struct fmc_sdram_timing {
	unsigned int load_to_active_delay;
	unsigned int exit_self_refresh_delay;
	unsigned int self_refresh_time;
	unsigned int row_cycle_delay;
	unsigned int write_recovery_time;
	unsigned int rp_delay;
	unsigned int rc_delay;
};

struct fmc_sdram {
	unsigned int num_bank;
	unsigned int column;
	unsigned int row;
	unsigned int data_width;
	unsigned int internal_bank;
	unsigned int cas;
	unsigned int write_protection;
	unsigned int clk_period;
	unsigned int read_burst;
	unsigned int read_pipe_delay;
	unsigned int conf_to_load;
	unsigned int refresh_count;
	struct fmc_sdram_timing *fmc_sdram_timing;
	struct fmc_sdram_cmd_config *fmc_sdram_cmd_config;
};

struct fmc {
	unsigned int mem_type;
	struct device dev;
};


#endif /* FMC_H */

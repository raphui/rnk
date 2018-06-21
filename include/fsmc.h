#ifndef FSMC_H
#define FSMC_H

struct fsmc_nor_write_timing {
	unsigned char acc_mode:2;
	unsigned char bus_turn:4;
	unsigned char data_phase_duration;
	unsigned char adress_hold_duration:4;
	unsigned char adress_setup_duration:4;
};

struct fsmc_nor_cs_timing {
	unsigned char acc_mode:2;
	unsigned char data_latency:4;
	unsigned char clk_div:4;
	unsigned char bus_turn:4;
	unsigned char data_phase_duration;
	unsigned char adress_hold_duration:4;
	unsigned char adress_setup_duration:4;
};

struct fsmc_nor {
	unsigned char bank:4;
	unsigned char bus_width:2;;
	unsigned char mem_type:2;
	struct fsmc_nor_write_timing fsmc_nor_w;
	struct fsmc_nor_cs_timing fsmc_nor_cs;
};

#endif /* FSMC_H */

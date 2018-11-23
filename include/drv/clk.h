#ifndef CLK_H
#define CLK_H

struct clk
{
	unsigned int id;
	unsigned int gated;
	unsigned int source_clk;
};

#endif /* CLK_H */

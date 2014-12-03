#ifndef UTILS_H
#define UTILS_H

//#define writel(r,v)	__writel(r,v)
//#define readl(r)	__readl(r)

void writel(unsigned int reg, unsigned int val);
unsigned int readl(unsigned int reg);

#endif /* UTILS_H */

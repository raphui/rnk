#ifndef UTILS_H
#define UTILS_H

static inline void writel( unsigned int reg , unsigned int val )
{
	unsigned int *p = ( unsigned int * )reg;
	
	asm volatile("str %[val], [%[reg]]"
				: : [reg]"r"(p), [val]"r"(val) );
}


static inline unsigned int readl( unsigned int reg )
{
	unsigned int *p = ( unsigned int * )reg;
	unsigned int val;
	
	asm volatile("ldr %[val], [%[reg]]"
				: [val]"=r"(val) : [reg]"r"(p) );

	return val;
}

static void delay( int count ) 
{
    asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
	     : : [count]"r"(count) : "cc");
}

#endif /* UTILS_H */
#ifndef UTILS_H
#define UTILS_H


void writel(unsigned int reg, unsigned int val);
unsigned int readl(unsigned int reg);
unsigned int next_power_of_2(unsigned int n);

#define ALIGN(x, a)	(((x) + (a) - 1) & ~((a) - 1))

#define min(x, y) ({                            \
	typeof(x) _min1 = (x);                  \
	typeof(y) _min2 = (y);                  \
	(void) (&_min1 == &_min2);              \
	_min1 < _min2 ? _min1 : _min2; })

#define max(x, y) ({                            \
	typeof(x) _max1 = (x);                  \
	typeof(y) _max2 = (y);                  \
	(void) (&_max1 == &_max2);              \
	 _max1 > _max2 ? _max1 : _max2; })


#define offset_of(type, member) __builtin_offsetof(type, member)

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:        the pointer to the member.
 * @type:       the type of the container struct this is embedded in.
 * @member:     the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({ \
    const typeof( ((type *)0)->member ) *__mptr = (ptr); \
    (type *)( (char *)__mptr - __builtin_offsetof(type,member) );})

/*
 * Divide positive or negative dividend by positive divisor and round
 * to closest integer. Result is undefined for negative divisors and
 * for negative dividends if the divisor variable type is unsigned.
 */
#define DIV_ROUND_CLOSEST(x, divisor)(                  \
{                                                       \
        typeof(x) __x = x;                              \
        typeof(divisor) __d = divisor;                  \
        (((typeof(x))-1) > 0 ||                         \
         ((typeof(divisor))-1) > 0 || (__x) > 0) ?      \
                (((__x) + ((__d) / 2)) / (__d)) :       \
                (((__x) - ((__d) / 2)) / (__d));        \
}                                                       \
)

#define DUMP_BINARY_FIELD(str, data, ptr, data_size, elem_size) ({ \
	int i;\
	printk("%s: ", str);\
	for (i = 0, ptr = (typeof(ptr))data; i < data_size; i += elem_size, ptr++)\
		printk("%x ", *p);\
	printk("\n");\
})

#define ROUND_DOWN(a,b) (((a) / (b)) * (b))

#endif /* UTILS_H */

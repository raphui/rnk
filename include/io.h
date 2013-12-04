#ifndef IO_H
#define IO_H

#include <stdint.h>

struct io_operations
{

	int ( *write )( const char *string );
};


struct io_operations io_op;


int printk( const char *fmt , ... );

#endif /* IO_H */

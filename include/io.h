#ifndef IO_H
#define IO_H

#include <stdint.h>

struct io_op
{

	int ( *write )( const char *string );
};


struct io_op io_functions;


int printk( const char *fmt , ... );

#endif /* IO_H */

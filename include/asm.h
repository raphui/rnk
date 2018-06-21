#ifndef ASM_H
#define ASM_H

#ifndef ENTRY
#define ENTRY( name ) \
	.globl name; \
	.ALIGN; \
	name:
#endif /* ENTRY */


#ifndef END
#define END( name ) \
	.size name, .-name
#endif /* END */

#endif /* ASM_H */

#ifndef RNK_LDS_H
#define RNK_LDS_H

#define INITCALLS			\
	KEEP(*(.initcall.0))			\
	KEEP(*(.initcall.1))			\
	KEEP(*(.initcall.2))			\
	KEEP(*(.initcall.3))			\
	KEEP(*(.initcall.4))			\
	KEEP(*(.initcall.5))			\
	KEEP(*(.initcall.6))			\
	KEEP(*(.initcall.7))			\
	KEEP(*(.initcall.8))			\
	KEEP(*(.initcall.9))			\
	KEEP(*(.initcall.10))			\
	KEEP(*(.initcall.11))			\
	KEEP(*(.initcall.12))			\
	KEEP(*(.initcall.13))			\
	KEEP(*(.initcall.14))

#define EXITCALLS			\
	KEEP(*(.exitcall.0))			\
	KEEP(*(.exitcall.1))			\
	KEEP(*(.exitcall.2))			\
	KEEP(*(.exitcall.3))			\
	KEEP(*(.exitcall.4))			\
	KEEP(*(.exitcall.5))			\
	KEEP(*(.exitcall.6))

#define KSYM				\
	KEEP(*(.ksym))

#endif /* RNK_LDS_H */

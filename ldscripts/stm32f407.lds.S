#include <rnk.lds.h>

MEMORY
{
	sram (W!RX) : ORIGIN = 0x20000000, LENGTH = 196K
	CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
	flash (RX) : ORIGIN = 0x08000000, LENGTH = 1024K
}

SECTIONS
{  
	.text :
	{
		. = ALIGN(4);
		_text = .;
		PROVIDE(stext = .);
		KEEP(*(.isr_vector))
		KEEP(*(.isr_vector_cmsis))
		*(.text .text.*)        
		*(.rodata .rodata.*)        
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)

		. = ALIGN(4);
		_etext = .;
		PROVIDE(etext = .);   
		_fini = . ;
		*(.fini)

	} >flash


	__rnk_initcalls_start = .;
	.rnk_initcalls : {
		INITCALLS
	} > flash
	__rnk_initcalls_end = .;


	__rnk_exitcalls_start = .;
	.rnk_exitcalls : {
		EXITCALLS
	} > flash 
	__rnk_exitcalls_end = .;

	__rnk_ksym_start = .;
	.rnk_ksym : {
		KSYM
	} > flash
	__rnk_ksym_end = .;

#ifdef CONFIG_UNWIND
	.ARM.extab :
	{
		*(.ARM.extab*)
	} >flash

	.ARM.exidx :
	{
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >flash
#endif

	_sidata = .;
	.data : AT (_sidata)
	{
		. = ALIGN(4);
		_sdata = .;
		*(.ramfunc .ramfunc.* .fastrun .fastrun.*)
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
		_edata = .;
	} >sram


	PROVIDE( flash_used_size = SIZEOF(.text) + SIZEOF(.data) + SIZEOF(.ARM.extab) + SIZEOF(.ARM.exidx) );

	.bss (NOLOAD) : {
		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .bss secion */
		_sbss = .;
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);        
		_ebss = .;
	} >sram

	_end = .;
	PROVIDE( _estack = _end + CONFIG_STACK_SIZE );
}


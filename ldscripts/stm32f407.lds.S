#include <rnk.lds.h>

MEMORY
{
	sram (W!RX) : ORIGIN = 0x20000000, LENGTH = 128K
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
#ifdef CONFIG_SW_ISR_TABLE
		KEEP(*(.isr_vector_cmsis))
#endif
		*(.text .text.*)        
		*(.rodata .rodata.*)        
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)

#ifdef CONFIG_INITCALL
		__rnk_initcalls_start = .;
		INITCALLS
		__rnk_initcalls_end = .;

		__rnk_exitcalls_start = .;
		EXITCALLS
		__rnk_exitcalls_end = .;
#endif

		. = ALIGN(4);
		_etext = .;
		_sidata = _etext;
		PROVIDE(etext = .);   
		_fini = . ;
		*(.fini)

	} >flash

	.data : AT (_etext)
	{
		. = ALIGN(4);
		_sdata = .;
		*(.ramfunc .ramfunc.* .fastrun .fastrun.*)
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
		_edata = .;
	} >flash

#ifdef CONFIG_UNWIND
	.ARM.extab : AT (_edata)
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

	end = .;
	PROVIDE( _estack = 0x20020000 );
}


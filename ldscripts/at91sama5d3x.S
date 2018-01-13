#include <rnk.lds.h>

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
	. = TEXT_BASE;

	. = ALIGN(4);
	.text      :
	{
		_stext = .;
		_text = .;
		KEEP(*(.text_exceptions*))
		*(.text .text.* .gnu.linkonce.t.*)
	}

	. = ALIGN(4);
	.rodata : { *(.rodata*) }

#ifdef CONFIG_UNWIND
	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	}

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	}
	__exidx_end = .;
#endif

	__rnk_initcalls_start = .;
	.rnk_initcalls : {
		INITCALLS
	}
	__rnk_initcalls_end = .;


	__rnk_exitcalls_start = .;
	.rnk_exitcalls : {
		EXITCALLS
	}
	__rnk_exitcalls_end = .;

	__rnk_ksym_start = .;
	.rnk_ksym : {
		KSYM
	}
	__rnk_ksym_end = .;

	_etext = .;			/* End of text and rodata section */
	_sdata = .;

	. = ALIGN(4);
	.data : { *(.data*) }

	_edata = .;

	. = ALIGN(4);
	__bss_start = .;
	.bss : { *(.bss*) }
	__bss_stop = .;
	_end = .;
	_rnk_image_size = __bss_start - TEXT_BASE;
	PROVIDE( _estack = _end + CONFIG_STACK_SIZE );
}

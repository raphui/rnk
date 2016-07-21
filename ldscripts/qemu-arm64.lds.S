#include <rnk.lds.h>
OUTPUT_FORMAT("elf64-littleaarch64", "elf64-littleaarch64", "elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
ENTRY(start)
SECTIONS
{
	. = TEXT_BASE;

	. = ALIGN(4);
	.text      :
	{
		_stext = .;
		_text = .;
		PROVIDE(stext = .);
		*(.text .text.*)
	}

	. = ALIGN(4);
	.rodata : { *(.rodata*) }

#ifdef CONFIG_UNWIND
	/*
	 * Stack unwinding tables
	 */
	. = ALIGN(8);
	.ARM.unwind_idx : {
		__start_unwind_idx = .;
		*(.ARM.exidx*)
		__stop_unwind_idx = .;
	}
	.ARM.unwind_tab : {
		__start_unwind_tab = .;
		*(.ARM.extab*)
		__stop_unwind_tab = .;
	}
#endif
	_etext = .;			/* End of text and rodata section */
	_sdata = .;
	. = ALIGN(4);
	.data : { *(.data*) }

	__rnk_initcalls_start = .;
	.rnk_initcalls : { INITCALLS }
	__rnk_initcalls_end = .;

	__rnk_exitcalls_start = .;
	.rnk_exitcalls : { EXITCALLS }
	__rnk_exitcalls_end = .;

	_edata = .;

	. = ALIGN(4);
	__bss_start = .;
	.bss : { *(.bss*) }
	__bss_stop = .;
	_end = .;
	_rnk_image_size = __bss_start - TEXT_BASE;
}

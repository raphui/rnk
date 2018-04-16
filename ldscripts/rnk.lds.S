#include <rnk.lds.h>

MEMORY
{
    rom (rx)  : ORIGIN = CONFIG_TEXT_BASE, LENGTH = CONFIG_TEXT_SIZE
    ram (rwx) : ORIGIN = CONFIG_RAM_BASE, LENGTH = CONFIG_RAM_SIZE
}
 
_eram = CONFIG_RAM_BASE + CONFIG_RAM_SIZE;

/* Section Definitions */
SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector .isr_vector.*))
        KEEP(*(.cm_isr_vector))
	KEEP(*(.isr_vector_cmsis))
        *(EXCLUDE_FILE(lib/*) .text EXCLUDE_FILE(lib/*) .text.* .gnu.linkonce.t.*)
        *(.glue_7t) *(.glue_7)
        *(EXCLUDE_FILE(lib/*) .rodata EXCLUDE_FILE(lib/*) .rodata* .gnu.linkonce.r.*)

	_slibtext = .;
	*(.lib/* .text .lib/* .text.*)
	*(.lib/* .rodata .lib/* .rodata*)
        . = ALIGN(4);
	_elibtext = .;

	. = ALIGN(4);
	_etext = .;
	PROVIDE(etext = .);
    } > rom
/* */
 
#ifdef CONFIG_UNWIND
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > rom
 
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > rom
    __exidx_end = .;
#endif

   __rnk_initcalls_start = .;
   .rnk_initcalls : {
	INITCALLS
   } > rom
   __rnk_initcalls_end = .;


   __rnk_exitcalls_start = .;
   .rnk_exitcalls : {
	EXITCALLS
   } > rom
   __rnk_exitcalls_end = .;

   __rnk_ksym_start = .;
   .rnk_ksym : {
	KSYM
   } > rom
   __rnk_ksym_end = .;
 
    _sidata = .;
    .data : AT (_sidata)
    {
        _sdata = .;
        *(EXCLUDE_FILE(lib/*) .data EXCLUDE_FILE(lib/*) .data.*)
	_slibdata = .;
	*(.lib/* .data .lib/* .data.*)
        . = ALIGN(4);
	_elibdata = .;
        _edata = . ;
    } > ram
 
    /* .bss section which is used for uninitialized data */
    .bss (NOLOAD) :
    {
        _sbss = . ;
	__bss_start__ = . ;
        *(EXCLUDE_FILE(lib/*) .bss EXCLUDE_FILE(lib/*) .bss.*)
        *(COMMON)
	_slibbss = .;
	*(.lib/* .bss .lib/* .bss.*)
        . = ALIGN(4);
	_elibbss = .;
        _ebss = . ;
	__bss_end__ = . ;
    } > ram
 
    /* stack section */
    .co_stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.co_stack .co_stack.*)
    } > ram
 
    . = ALIGN(4);
    _end = . ;

    PROVIDE(_sstack = _eram - CONFIG_STACK_SIZE);
    PROVIDE( _estack = _eram );
}

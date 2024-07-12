#include <rnk.lds.h>

MEMORY
{
#ifdef CONFIG_LR1110_UPDATE_APP
    flash(rx)  : ORIGIN = 0x08000000, LENGTH = 0x00040000
#endif
    rom (rx)  : ORIGIN = CONFIG_TEXT_BASE, LENGTH = CONFIG_TEXT_SIZE
    ram (rwx) : ORIGIN = CONFIG_RAM_BASE + CONFIG_STACK_SIZE, LENGTH = CONFIG_RAM_SIZE - CONFIG_STACK_SIZE
}
 
_eram = CONFIG_RAM_BASE + CONFIG_RAM_SIZE;

/* Section Definitions */
SECTIONS
{
#ifdef CONFIG_LR1110_UPDATE_APP
    .blob :
    {
        *(.blob)
    } > flash
#endif
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector .isr_vector.*))
        KEEP(*(.cm_isr_vector))
	KEEP(*(.isr_vector_cmsis))
        *(EXCLUDE_FILE(*objs/lib/*) .text EXCLUDE_FILE(*objs/lib/*) .text.* .gnu.linkonce.t.*)
        *(.glue_7t) *(.glue_7)
        *(EXCLUDE_FILE(*objs/lib/*) .rodata EXCLUDE_FILE(*objs/lib/*) .rodata* .gnu.linkonce.r.*)

	_slibtext = .;
	*(*objs/lib/* .text *objs/lib/* .text.*)
	*(*objs/lib/* .rodata *objs/lib/* .rodata*)
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
        *(EXCLUDE_FILE(*objs/lib/*) .data EXCLUDE_FILE(*objs/lib/*) .data.*)
	_slibdata = .;
	*(*objs/lib/* .data *objs/lib/* .data.*)
        . = ALIGN(4);
	_elibdata = .;
        _edata = . ;
    } > ram
 
    /* .bss section which is used for uninitialized data */
    .bss (NOLOAD) :
    {
        _sbss = . ;
	__bss_start__ = . ;
        *(EXCLUDE_FILE(*objs/lib/*) .bss EXCLUDE_FILE(*objs/lib/*) .bss.*)
        *(COMMON)
	_slibbss = .;
	*(*objs/lib/* .bss *objs/lib/* .bss.*)
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

    PROVIDE(_sstack = CONFIG_RAM_BASE);
    PROVIDE(_estack = CONFIG_RAM_BASE + CONFIG_STACK_SIZE);
}

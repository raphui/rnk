OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
/* Internal Memory Map*/
MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 0x00100000
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00002000
    ccm (rwx) : ORIGIN = 0x10000000, LENGTH = 0x00010000
}
 
_eram = 0x20000000 + 0x00020000;

/* Section Definitions */
SECTIONS
{
    /* collect all uninitialized .ccm sections */
	/* A directed static allocation would look like this :
		volatile uint8_t buff0[16384] __attribute__((section(".ccm")));   */
	.ccm (NOLOAD) :
    {
        . = ALIGN(4);
        *(.ccm)
        *(.ccm.*)
        . = ALIGN(4);
    } > ccm
 
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector .isr_vector.*))
        *(.text .text.* .gnu.linkonce.t.*)
        *(.glue_7t) *(.glue_7)
        *(.rodata .rodata* .gnu.linkonce.r.*)
	__rnk_initcalls_start = .;
	KEEP(*(.initcall.0))
	KEEP(*(.initcall.1))
	KEEP(*(.initcall.2))
	KEEP(*(.initcall.3))
	KEEP(*(.initcall.4))
	KEEP(*(.initcall.5))
	KEEP(*(.initcall.6))
	KEEP(*(.initcall.7))
	KEEP(*(.initcall.8))
	KEEP(*(.initcall.9))
	KEEP(*(.initcall.10))
	KEEP(*(.initcall.11))
	KEEP(*(.initcall.12))
	KEEP(*(.initcall.13))
	KEEP(*(.initcall.14))
	__rnk_initcalls_end = .;

	__rnk_exitcalls_start = .;
	KEEP(*(.exitcall.0))
	KEEP(*(.exitcall.1))
	KEEP(*(.exitcall.2))
	KEEP(*(.exitcall.3))
	KEEP(*(.exitcall.4))
	KEEP(*(.exitcall.5))
	KEEP(*(.exitcall.6))
	__rnk_exitcalls_end = .;
    } > rom
 
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
 
    . = ALIGN(4);
    _etext = .;
    _sidata = .;
 
    .data : AT (_etext)
    {
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = . ;
    } > ram
 
    /* .bss section which is used for uninitialized data */
    .bss (NOLOAD) :
    {
        _sbss = . ;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = . ;
    } > ram
 
    /* stack section */
    .co_stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.co_stack .co_stack.*)
    } > ram
 
    . = ALIGN(4);
    _end = . ;
    PROVIDE( _estack = 0x20020000 );
}

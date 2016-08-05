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
        *(.text .text.*)        
        *(.rodata .rodata.*)        
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
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


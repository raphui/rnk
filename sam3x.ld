/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*------------------------------------------------------------------------------
 *      Linker script for running in internal PSRAM on the AT91SAM3U4
 *----------------------------------------------------------------------------*/

STACK_SIZE =  4K - 4;

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(ResetException)

/* Vector Definition */
/* vector is put at very begin adress of SRAM0, the preserved size is 0x100 */
_vect_start = 0x20000000;

/* Memory Spaces Definitions */
MEMORY
{
	rom  (W!RX) 	: ORIGIN = 0x00100000, LENGTH = 0x00010000 /* Flash, 64K */
	flash  (W!RX) 	: ORIGIN = 0x00080000, LENGTH = 0x00080000 /* Flash, 512K */
	sram (W!RX) 	: ORIGIN = 0x20070100, LENGTH = 0x00016F00 /* sram, 92K */
	sramstack (W!RX): ORIGIN = 0x20080000, LENGTH = 0x00001000 /* sram, 4K */
	sdram (W!RX)  	: ORIGIN = 0x70000000, LENGTH = 0x02000000 /* SDRAM, 32M */
}

SECTIONS
{
    .text : 
    { 
        . = ALIGN(4);
        _sfixed = .;
        _svectorrelocate = .;     
        KEEP(*(.vectors .vectors.*))
        _evectorrelocate = .;
		*(.text*)
        *(.rodata*)
	    *(.glue_7);
        *(.glue_7t);
        . = ALIGN(4);
        _efixed = .;            /* End of text section */
    } > flash
   
    .relocate : AT (_efixed)
    {
    	. = ALIGN(4);
    	_srelocate = .;
        *(.ramfunc)
        *(.data)
        *(COMMON)
        . = ALIGN(4);
        _erelocate = .;
    } >sram

    .ARM.extab : AT (_efixed + SIZEOF(.relocate))
    {
    	. = ALIGN(4);
	    __extab_start = .;
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    	. = ALIGN(4);
	    __extab_end = .;
    }>sram
    
    .ARM.exidx : AT (_efixed + SIZEOF(.relocate) + SIZEOF(.ARM.extab))
    {
    	. = ALIGN(4);
		__exidx_start = .;
    	*(.ARM.exidx* .gnu.linkonce.armexidx.*)
    	. = ALIGN(4);
    	__exidx_end = .;
    }>sram
    .bss (NOLOAD) : {
        . = ALIGN(4);
        _szero = .;
        *(.bss)
        . = ALIGN(4);
        _ezero = .;
    } >sram
    
    .nandinfo (NOLOAD) : 
    {
        . = ALIGN(4);
        _bssszero = .;
        *(.nandinfo)
        . = ALIGN(4);
        _ezero = .;
    } >sram
    
    .image (NOLOAD) : 
    {
        . = ALIGN(4);
        *(.image)
        . = ALIGN(4);
    } >sdram
    
  /* Stack in the end of SRAM1 */  
  .stack (NOLOAD):
  {
   	  _stack_end = .;
      . += STACK_SIZE;
      . = ALIGN(4);
      _stack_start = .;
  } > sramstack
      
  . = ALIGN(4); 
  _end = . ; 

}


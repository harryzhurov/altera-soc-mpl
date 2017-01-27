/******************************************************************************
*
* Copyright 2014 Altera Corporation. All Rights Reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* 
******************************************************************************/

/*
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/mpl_gcc.lds#2 $
 */

MEMORY
{
   prog_mem (rwx) : ORIGIN = 0xffff0000, LENGTH = 64K
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(mpl_vectors)

SECTIONS
{
    . = ORIGIN(prog_mem);
    .text : { *(.text) } > prog_mem             /* (.text) usually contains the code*/
    . = ALIGN(4);

    .rodata : { *(.rodata) } > prog_mem         /* (.rodata) contains data like constant vars */
    . = ALIGN(4);

    .data : { *(.data) } > prog_mem             /* initialized data; usually contains initialized variables */    
    . = ALIGN(4);	

    __bss_start = .;
    .bss : { *(.bss) } > prog_mem               /* usually contains non-initialized data. */
    . = ALIGN(4);
    __bss_end = .;
	
    .stack : { 
        . = ALIGN(4);
        user_stack_start = .;
        user_stack_end = user_stack_start + 1024;  /* main stack size = 1K (1024 byets) */ 
        . = user_stack_end + 4;
    } > prog_mem
    . = ALIGN(4);
}

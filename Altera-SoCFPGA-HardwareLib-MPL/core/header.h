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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/header.h#1 $
 */

#ifndef _HEADER_H
#define _HEADER_H

#include "hwlib.h"

#define IMG_HDR_SZ  0x40

// MKIMAGE name length = 32
// NOTE: all data is stored on flash in big-endian format
typedef struct img_header {
    uint32_t    magic;          // magic number to validate image
    uint32_t    hcrc;           // img header crc checksum
    uint32_t    timestamp;      // timestamp
    uint32_t    img_size;       // image data size
    uint32_t    load_addr;      // data load address
    uint32_t    entry_point;    // entry point address
    uint32_t    dcrc;           // img data crc checksum
    uint8_t     os;             // operating system
    uint8_t     arch;           // cpu architecture
    uint8_t     type;           // image type
    uint8_t     comp;           // compression type
    uint8_t     name[32];       // image name
} img_header_t;

ALT_STATUS_CODE header_validate(img_header_t * pheader);

#endif  /* _HEADER_H */

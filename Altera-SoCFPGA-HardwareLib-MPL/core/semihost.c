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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/semihost.c#1 $
 */

#include "hwlib.h"
#include "alt_printf.h"

int fd;
#ifdef USE_WRITE
#define LOGBUFF_SIZE	80
static int logbuffoffset=0;
static char logbuff[LOGBUFF_SIZE];

void alt_log_flush()
{
    if(logbuffoffset)
      _sys_write(fd, logbuff, logbuffoffset);
    logbuffoffset = 0;
}
#endif

void upc(void *p_port, char toprint)
{
#ifdef USE_WRITE
    logbuff[logbuffoffset++] = toprint;
    if(logbuffoffset >= LOGBUFF_SIZE ||
       toprint == '\n')
    {  
       alt_log_flush();
    }
#else
    _ttywrch(toprint); // Warning, Slow, but it works!
#endif
}

FILEOP term0_st = {upc, NULL};

ALT_STATUS_CODE alt_log_init(void)
{
    fd = _sys_open(fd, 0);
    return ALT_E_SUCCESS;
}

void alt_log_done(void)
{
#ifdef USE_WRITE
    alt_log_flush();
#endif
    _sys_close(0);
}


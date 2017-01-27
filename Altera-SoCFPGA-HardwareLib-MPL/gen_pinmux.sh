#*****************************************************************************
#
# Copyright 2014-2016 Altera Corporation. All Rights Reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
#*****************************************************************************

# The purpose of this file is the generate the code for the pinmux configuration from
# a u-boot dts file for Arria10

filename=$1
filetype="${filename##*.}"

echo "#include <socal/socal.h>"
echo "#include <socal/hps.h>"
echo "#include <alt_pinmux.h>"
echo ""
echo "void sysmgr_pinmux_init(void)"
echo "{"
echo "    uint32_t *addr;"

function printlist {
#dtc -o - -O dts -I $filetype $filename | grep -sw -A4 $1
count=0
isaddr=true
for a in $(dtc -o - -O dts -I $filetype $filename | grep -sw -A4 $1 | grep "pinctrl-single,pins" | awk -F = '{print $2}' | sed -e 's/<//' -e 's/>;//'); do 
        if [ "$isaddr" = true ]
	then
	   printf '    addr[%s/4] = ' $a;
	   isaddr=false
	else
	   printf '%s;\n' $a;
	   isaddr=true
	fi
	(( count+=2 ))
done
}


echo ""
echo "    addr = ALT_PINMUX_SHARED_3V_IO_GRP_ADDR; /* 0xffd07000 */"
printlist shared

echo ""
echo "    addr = ALT_PINMUX_DCTD_IO_GRP_ADDR; /* 0xffd07200 */"
printlist dedicated

echo ""
echo "    addr = ALT_PINMUX_DCTD_IO_CFG_BANK_ADDR; /* 0xffd07300 */"
printlist dedicated_cfg

echo ""
echo "    addr = ALT_PINMUX_SHARED_3V_IO_GRP_ADDR; /* 0xffd07400 */"
printlist fpga

echo "}"


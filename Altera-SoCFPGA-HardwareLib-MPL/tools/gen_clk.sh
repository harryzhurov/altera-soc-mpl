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

# The purpose of this file is the generate the code for the clock configuration from
# a u-boot dts file for Arria10

filename=$1
filetype="${filename##*.}"

function printheader {
  echo "#include <alt_clock_manager.h>"
  echo ""
  echo "void get_clockdata(CLOCK_SOURCE_CONFIG *psrc, CLOCK_MANAGER_CONFIG *pMgr)"
  echo "{"
}
printheader

function getval {
dtc -o - -O dts -I $filetype $filename | grep -sw -A4 $1 | grep $2 | awk -F = '{print $2}' | sed -e 's/<//' -e 's/>;//'
}

clk=$(getval altera_arria10_hps_eosc1 clock-frequency)
printf "    psrc->clk_freq_of_eosc1              = %d;\n" $(($clk))

clk=$(getval altera_arria10_hps_cb_intosc_ls clock-frequency)
printf "    psrc->clk_freq_of_cb_intosc_ls = %d;\n" $(($clk))

clk=$(getval altera_arria10_hps_f2h_free clock-frequency)
printf "    psrc->clk_freq_of_f2h_free           = %d;\n" $(($clk))




function getclks {
dtc -o - -O dts -I $filetype $filename | grep -sw -A$2 $1 | grep -v $1 | sed -r -e 's/-/_/' -e 's/<//g' -e 's/>//g' -e 's/^\s+//' -e 's/^/    pMgr->'$1'./'
}

echo ""
getclks mainpll 24
echo ""
getclks perpll 21
echo ""
getclks alteragrp 2

echo "}"


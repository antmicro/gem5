# Copyright (c) 2023 Antmicro Ltd.
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2005-2007 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.proxy import *

from m5.util.fdthelper import *
from m5.defines import buildEnv

from m5.SimObject import SimObject
from m5.objects.Device import DmaDevice
from m5.objects.Serial import SerialDevice
from m5.objects.I2C import I2CDevice
from m5.objects.PS2 import PS2Device
from m5.objects.XlsPlatform import XlsPlatform


# class XlsBind(SimObject):
#    type = "XlsBind"
#    cxx_header = "dev/riscv/xls.hh"
#    cxx_class = "gem5::XlsBind"
#    channel = Param.String("", "XLS Channel name")
#    offset = Param.Addr(0x0, "Offset at which the channel should be mapped")


class XlsDev(DmaDevice):
    type = "XlsDev"
    cxx_header = "dev/cosim/xls.hh"
    cxx_class = "gem5::XlsDev"
    byte_order = Param.ByteOrder("little", "Device byte order")
    pio_addr = Param.Addr(0x0, "Device PIO address")
    pio_size = Param.Addr(0x4, "Size of address range")
    xls_plugin_linux = Param.String("", "Path to xls .so plugin for Linux")
    config = Param.String("", "Path to peripheral configuration")
    platform = Param.XlsPlatform(
        Parent.any, "XlsPlatform this device is part of."
    )
    device_id = Param.UInt32(0, "XLS Device ID")


#    bindings = VectorParam.XlsBind([], "XLS channel mappings")

# Copyright (c) 2023 Antmicro Ltd.
# All rights reserved
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


from m5.objects.XlsPlatform import XlsPlatform
from m5.objects.Clint import Clint
from m5.objects.Plic import Plic
from m5.objects.Uart import SimpleUart
from m5.objects.Xls import XlsDev
from m5.objects.Terminal import Terminal
from m5.params import *
from m5.proxy import *


class XlsDemoBase(XlsPlatform):
    type = "XlsDemoBase"
    cxx_header = "dev/cosim/XlsDemo.hh"
    cxx_class = "gem5::XlsDemoBase"

    # PLIC
    plic = Param.PlicBase(NULL, "PLIC")

    # Int source ID to redirect console interrupts to
    # Set to 0 if using a pci interrupt for Uart instead
    uart_int_id = Param.Int(0, "PLIC Uart interrupt ID")

    xls_so_path = Param.String(
        "path/to/libgem5_xls_plugin.so", "Path to XLS plugin"
    )
    xls_config_path = Param.String(
        "path/to/xls_config.textprotoo", "Path to XLS device configuration"
    )


class XlsDemo(XlsDemoBase):

    # Unfortunately, we need to go with U54MC and its plic/clint because
    # VexRiscV is not supported

    # We don't need CLINT dor this one
    # clint = Clint(pio_addr=0x2000000)
    plic = Plic(pio_addr=0xC000000)

    uart = SimpleUart(pio_addr=0xE0001800)

    xls_dev = XlsDev(
        pio_addr=0x070000000,
        xls_plugin_linux="path/to/libgem5_xls_plugin.so",
        config="path/to/xls_config.textproto",
        device_id=0,
    )

    uart_int_id = 0xA
    terminal = Terminal()

    def _on_chip_devices(self):
        """Returns a list of on-chip peripherals"""
        return [self.plic]

    def _off_chip_devices(self):
        """Returns a list of off-chip peripherals"""
        return [self.uart, self.xls_dev]

    def attachXlsDev(self):
        self.xls_dev.xls_plugin_linux = self.xls_so_path
        self.xls_dev.config = self.xls_config_path

    def attachPlic(self):
        """Count and set number of PLIC interrupt sources"""
        plic_max_src = self.uart_int_id

        for device in self._off_chip_devices():
            if hasattr(device, "interrupt_id"):
                if device.interrupt_id > plic_max_src:
                    plic_max_src = device.interrupt_id

        self.plic.n_src = plic_max_src + 1

    def attachOnChipIO(self, bus):
        """Attach on-chip IO devices, needs modification
        to support DMA
        """
        for device in self._on_chip_devices():
            device.pio = bus.mem_side_ports

    def attachOffChipIO(self, bus):
        """Attach off-chip IO devices, needs modification
        to support DMA
        """
        for device in self._off_chip_devices():
            device.pio = bus.mem_side_ports

        self.xls_dev.dma = bus.cpu_side_ports

    def setNumCores(self, num_cpu):
        """Sets the PLIC and CLINT to have the right number of threads and
        contexts. Assumes that the cores have a single hardware thread.
        """
        self.plic.n_contexts = num_cpu * 2
        # self.clint.num_threads = num_cpu

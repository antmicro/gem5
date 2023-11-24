import m5

from m5.objects import *

from argparse import ArgumentParser

from gem5.utils.requires import requires
from gem5.isas import ISA

parser = ArgumentParser()
parser.add_argument(
    "--firmware", type=str, required=True, help="Path to .elf binary to run"
)
parser.add_argument(
    "--xls-plugin", type=str, required=True, help="Path to .so XLS gem5 plugin"
)
parser.add_argument(
    "--xls-config", type=str, help="Path to XLS config for co-simulation"
)
parser.add_argument("--gdb", action="store_true", help="Wait for GDB")
args = parser.parse_args()

requires(isa_required=ISA.RISCV)

system = System()
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing"
system.mem_ranges = [AddrRange(start=0x40000000, size=0x10000000)]  # 256MB

system.cpu = RiscvTimingSimpleCPU()
system.cpu.ArchISA.riscv_type = "RV32"

system.membus = SystemXBar()
# system.iobus = IOXBar()

system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports

system.cpu.createInterruptController()

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.device_size = "16MB"
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

system.workload = RiscvBareMetal()
system.workload.bootloader = args.firmware
system.workload.wait_for_remote_gdb = args.gdb

system.platform = XlsDemo(
    xls_so_path=args.xls_plugin,
    xls_config_path=args.xls_config,
)
system.platform.attachXlsDev()
system.platform.attachPlic()
system.platform.setNumCores(1)
system.platform.attachOnChipIO(system.membus)
system.platform.attachOffChipIO(system.membus)

system.system_port = system.membus.cpu_side_ports

system.cpu.createThreads()

root = Root(full_system=True, system=system)
m5.instantiate()

print("Starting simulation")
exit_event = m5.simulate()

print(
    "Exiting @ tick {} because {}".format(m5.curTick(), exit_event.getCause())
)

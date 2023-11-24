/*
 * Copyright (c) 2023 Antmicro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file
 * Implements XLS device for cosimulation
 */

#include "dev/cosim/xls.hh"

#include <dlfcn.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/XlsDev.hh"
#include "dev/dma_device.hh"
#include "dev/xls_platform.hh"
#include "params/XlsDev.hh"
#include "sim/cur_tick.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

// Linking seems to break if I put this under `class XlsLibSyms`:
// warning: relocation against `*********' in read-only section
static std::string XlsLibSymslinuxPath;

// TODO: Support other OSes
#define _C __attribute__((sysv_abi))

class XlsLibSyms
{
  private:
    // C library symbols

    using C_ErrorOps = Xls_C_ErrorOps;

    using C_XlsPeripheralBuildCtxH = void *;
    using C_XlsIRDesignH = void *;
    using C_XlsPeripheralConfigH = void *;
    using C_XlsPeripheralH = void *;
    using C_XlsIRSingleValueH = void *;
    using C_XlsIRStreamH = void *;

    void completeXfer(const C_OnRequestCompletionCB& callback) {
        callback.complete(callback.ctx);
    }

    struct C_XlsCallbacks
    {
        void *ctx;
        int (_C *xls_dmaMemToDev)(void *ctx, size_t addr, size_t count,
                                  uint8_t* buf,
                                  C_OnRequestCompletionCB response_ops);
        int (_C *xls_dmaDevToMem)(void *ctx, size_t addr, size_t count,
                                  const uint8_t* buf,
                                  C_OnRequestCompletionCB response_ops);
        int (_C *xls_requestIRQ)(void* ctx, size_t num, int state);
        void (_C *xls_log)(void* ctx, int severity, const char* message);
    };

    using C_BindSingleValueToMemCB =
        void (_C *)(void *, C_XlsIRSingleValueH, size_t);
    using C_BindStreamToMemCB = void (_C *)(void *, C_XlsIRStreamH, size_t);
    using C_BindStreamToDMACB = void (_C *)(void *, C_XlsIRStreamH, size_t);
    using C_BindStreamToAXIDMACB =
        void (_C *)(void *, C_XlsIRStreamH, size_t);

    int (_C *xls_initCallbacks)(C_XlsCallbacks);

    C_XlsPeripheralH (_C *xls_setupPeripheral) (const char*, C_ErrorOps*);
    int (_C *xls_resetPeripheral)(C_XlsPeripheralH, C_ErrorOps*);
    void (_C *xls_destroyPeripheral)(C_XlsPeripheralH, C_ErrorOps*);
    size_t (_C *xls_getPeripheralSize)(C_XlsPeripheralH, C_ErrorOps*);
    int (_C *xls_updatePeripheral)(C_XlsPeripheralH, C_ErrorOps*);
    int (_C *xls_readByte)
        (C_XlsPeripheralH, uint64_t, uint8_t*, C_ErrorOps*);
    int (_C *xls_readWord)
        (C_XlsPeripheralH, uint64_t, uint16_t*, C_ErrorOps*);
    int (_C *xls_readDWord)
        (C_XlsPeripheralH, uint64_t, uint32_t*, C_ErrorOps*);
    int (_C *xls_readQWord)
        (C_XlsPeripheralH, uint64_t, uint64_t*, C_ErrorOps*);
    int (_C *xls_writeByte)
        (C_XlsPeripheralH, uint64_t, uint8_t, C_ErrorOps*);
    int (_C *xls_writeWord)
        (C_XlsPeripheralH, uint64_t, uint16_t, C_ErrorOps*);
    int (_C *xls_writeDWord)
        (C_XlsPeripheralH, uint64_t, uint32_t, C_ErrorOps*);
    int (_C *xls_writeQWord)
        (C_XlsPeripheralH, uint64_t, uint64_t, C_ErrorOps*);

    friend class XlsPeripheral;
    friend class XlsStream;
    friend class XlsPeripheralBuilder;

    bool loaded = false;
    C_ErrorOps error_ops = { nullptr, nullptr };

    template <typename SymType>
    void
    loadSymbolLinux(void *dlhandle, SymType *symp, const char *name)
    {
        *symp = (SymType)dlsym(dlhandle, name);
        fatal_if(!(*symp), "Failed to load xls.so symbol `%s`\n", name);
    }

#define LOAD_SYMBOL_LINUX(dlhandle, sym) \
    loadSymbolLinux((dlhandle), &(sym), #sym)

    void
    loadDynLibLinux()
    {
        void *dlhandle =
            dlopen(XlsLibSymslinuxPath.c_str(), RTLD_LAZY | RTLD_DEEPBIND);
        fatal_if(!dlhandle, "Failed to open XLS dynamic library: `%s`\n"
                            "Reason: %s\n",
                 XlsLibSymslinuxPath.c_str(), dlerror());

        LOAD_SYMBOL_LINUX(dlhandle, xls_initCallbacks);
        LOAD_SYMBOL_LINUX(dlhandle, xls_setupPeripheral);
        LOAD_SYMBOL_LINUX(dlhandle, xls_resetPeripheral);
        LOAD_SYMBOL_LINUX(dlhandle, xls_destroyPeripheral);
        LOAD_SYMBOL_LINUX(dlhandle, xls_getPeripheralSize);
        LOAD_SYMBOL_LINUX(dlhandle, xls_updatePeripheral);
        LOAD_SYMBOL_LINUX(dlhandle, xls_readByte);
        LOAD_SYMBOL_LINUX(dlhandle, xls_readWord);
        LOAD_SYMBOL_LINUX(dlhandle, xls_readDWord);
        LOAD_SYMBOL_LINUX(dlhandle, xls_readQWord);
        LOAD_SYMBOL_LINUX(dlhandle, xls_writeByte);
        LOAD_SYMBOL_LINUX(dlhandle, xls_writeWord);
        LOAD_SYMBOL_LINUX(dlhandle, xls_writeDWord);
        LOAD_SYMBOL_LINUX(dlhandle, xls_writeQWord);
    }

  public:
    static void
    setLinuxPath(std::string path)
    {
        static std::string dl_linuxPath;
        if ((XlsLibSymslinuxPath != "") && (XlsLibSymslinuxPath != path)) {
            warn("Path for linux XLS .so is already set and differs from "
                 "the requested path\n");
        }
        XlsLibSymslinuxPath = path;
    }

    XlsPeripheral setupPeripheral(const std::string& config_path,
                                  DmaDevice &dma_dev, XlsPlatform &platform,
                                  uint32_t device_id) {
        warn("before xls_setupPeripheral call\n");
        C_XlsPeripheralH handle =
            xls_setupPeripheral(config_path.c_str(), &error_ops);
        warn("after xls_setupPeripheral call\n");
        if (!handle) {
            fatal("Failed to build a peripheral with configuration: \"s\"",
                  config_path.c_str());
        }
        return XlsPeripheral(handle, dma_dev, platform, device_id);
    }

    static XlsLibSyms&
    get()
    {
        static XlsLibSyms syms;
        if (!syms.loaded) {
            syms.loadDynLibLinux();
            syms.error_ops = C_ErrorOps {
                .ctx = &syms,
                .reportError =
                    [](void *ctx, uint8_t severity, const char *message) {
                        fatal("XLS error (%d): %s\n", severity, message);
                    },
            };
        }
        return syms;
    }
};

template <typename T, typename F>
T
XlsPeripheral::xlsTry(F procedure)
{
    // Will be updated in `error_ops` handler in case an error occurs.
    gotError = false;

    procedure();

    if (gotError) {
        warn("XLS operation failed with the following errors:\n");
        for (int i = 0; i < errors.size(); ++i) {
            warn("[%02d]: %s\n", i, errors[i]);
        }
        fatal("Exiting due to XLS errors.\n");
    }
}

void
XlsPeripheral::reset()
{
    XlsLibSyms::get().xls_resetPeripheral(handle, &error_ops);
}

size_t
XlsPeripheral::getPeripheralSize()
{
    return XlsLibSyms::get().xls_getPeripheralSize(handle, nullptr);
}

XlsPeripheral::XlsPeripheral(void *handle_, DmaDevice &dma_dev_,
                             XlsPlatform &platform_, uint32_t device_id_) :
    handle(handle_),
    dma_dev(dma_dev_),
    platform(platform_),
    device_id(device_id_)
{
    error_ops = Xls_C_ErrorOps {
        .ctx = this,
        .reportError = [](void *vthis, uint8_t severity, const char *message) {
            XlsPeripheral *me = static_cast<decltype(this)>(vthis);
            me->errors.push_back(
                "[" + std::to_string(severity) + "]" + std::string(message)
            );
            me->gotError = true;
        }
    };

    XlsLibSyms::get().xls_initCallbacks(XlsLibSyms::C_XlsCallbacks {
        .ctx = this,
        .xls_dmaMemToDev = [](void *ctx, size_t addr, size_t count,
                uint8_t* buf, C_OnRequestCompletionCB response_ops) -> int {
            auto *me = static_cast<decltype(this)>(ctx);
            DPRINTFS(XlsDev, me,
                     "SIM->XLS [0x%08x : 0x%08x] (%d bytes to %p)\n",
                     addr, addr + count, count, buf);
            auto *evt = new XferEvent(*me, response_ops, addr);
            me->dma_dev.dmaRead(addr, count, evt, buf, 1);
            return 0;
        },
        .xls_dmaDevToMem = [](void *ctx, size_t addr, size_t count,
                const uint8_t* buf,
                C_OnRequestCompletionCB response_ops) -> int {
            auto *me = static_cast<decltype(this)>(ctx);
            DPRINTFS(XlsDev, me,
                     "XLS->SIM [%08x : %08x] (%d bytes to %p)\n",
                     addr, addr + count, count, buf);
            auto *evt = new XferEvent(*me, response_ops, addr);
            // gem5 calls dmaPort.dmaAction(MemCmd::WriteReq, ...).
            // This won't modify the bffer, but other actions might.
            uint8_t *buf_cc = const_cast<uint8_t *>(buf);
            me->dma_dev.dmaWrite(addr, count, evt, buf_cc, 1);
            return 0;
        },
        .xls_requestIRQ = [](void *ctx, size_t num, int state) -> int {
            if (num != 0) {
                warn("Unsupported: Received IRQ != 0 from XLS."
                     "This will treated as IRQ0.\n");
            }
            static_cast<decltype(this)>(ctx)->interrupt(state != 0);
            return 0;
        },
        .xls_log = [](void *ctx, int severity, const char* message) {
            static_cast<decltype(this)>(ctx)->log(severity, message);
        },
    });
};

XlsPeripheral::~XlsPeripheral()
{
    if (handle == nullptr) return;
    xlsTry<void>([&] {
        XlsLibSyms::get().xls_destroyPeripheral(handle, &error_ops);
    });
}

void
XlsPeripheral::update()
{
    XlsLibSyms::get().xls_updatePeripheral(handle, &error_ops);
}

int
XlsPeripheral::read(void *data, size_t addr, size_t bytes)
{
    XlsLibSyms &xls = XlsLibSyms::get();

    size_t error, bytes_read;
    char *cursor = (char *)data;
    do {
        xlsTry<void>([&]() {
            if (bytes & 0x01) {
                error = xls.xls_readByte(handle, addr, (uint8_t*)cursor,
                                         &error_ops);
                bytes_read = 1;
            } else if (bytes & 0x02) {
                error = xls.xls_readWord(handle, addr, (uint16_t*)cursor,
                                         &error_ops);
                bytes_read = 2;
            } else if (bytes & 0x04) {
                error = xls.xls_readDWord(handle, addr, (uint32_t*)cursor,
                                          &error_ops);
                bytes_read = 4;
            } else if (bytes & 0x08) {
                error = xls.xls_readQWord(handle, addr, (uint64_t*)cursor,
                                          &error_ops);
                bytes_read = 8;
            }
        });

        cursor += bytes_read;
        bytes -= bytes_read;
    } while (!error && bytes);

    return error;
}

int XlsPeripheral::write(const void *data, size_t addr, size_t bytes)
{
    XlsLibSyms &xls = XlsLibSyms::get();

    size_t error, bytes_read;
    char *cursor = (char *)data;
    do {
        xlsTry<void>([&]() {
            if (bytes & 0x01) {
                error = xls.xls_writeByte(handle, addr, *(uint8_t*)cursor,
                                         &error_ops);
                bytes_read = 1;
            } else if (bytes & 0x02) {
                error = xls.xls_writeWord(handle, addr, *(uint16_t*)cursor,
                                         &error_ops);
                bytes_read = 2;
            } else if (bytes & 0x04) {
                error = xls.xls_writeDWord(handle, addr, *(uint32_t*)cursor,
                                           &error_ops);
                bytes_read = 4;
            } else if (bytes & 0x08) {
                error = xls.xls_writeQWord(handle, addr, *(uint64_t*)cursor,
                                           &error_ops);
                bytes_read = 8;
            }
        });

        cursor += bytes_read;
        bytes -= bytes_read;
    } while (!error && bytes);

    return error;
}

void
XlsPeripheral::log(size_t severity, const char *message) const
{
    const char *severity_str;
    switch (severity) {
      case 0:  severity_str = "info"; break;
      case 1:  severity_str = "warn"; break;
      case 2:  severity_str = "error"; break;
      default: severity_str = "unknown"; break;
    }

    DPRINTF(XlsDev, "\n            [XLS/%s]: %s\n", severity_str, message);
}

void
XlsPeripheral::interrupt(bool state)
{
    if (state) {
        DPRINTF(XlsDev, "Posting an interrupt.\n");
        platform.postXlsInt(device_id);
    } else {
        DPRINTF(XlsDev, "Clearing an interrupt.\n");
        platform.clearXlsInt(device_id);
    }
}

XlsPeripheral::XferEvent::XferEvent(
        XlsPeripheral &xlsp_, C_OnRequestCompletionCB response_ops_, Addr at_)
    : xlsp(xlsp_), response_ops(response_ops_), completed(false), at(at_)
{
}

void
XlsPeripheral::XferEvent::process()
{
    DPRINTF(XlsDev, "Completing request at 0x%08lx\n", at);
    response_ops.complete(response_ops.ctx);
    completed = true;
    delete this; // hehe
}


XlsPeripheral
XlsDev::createPeripheral(const Params &p, DmaDevice &dma_dev)
{
    XlsLibSyms::setLinuxPath(p.xls_plugin_linux);
    XlsLibSyms &xls = XlsLibSyms::get();

    return xls.setupPeripheral(p.config, dma_dev, *p.platform, p.device_id);
    DPRINTFR(XlsDev, "Peripheral has been set-up\n");
}

XlsDev::XlsDev(const Params &p) :
    DmaDevice(p),
    pioAddr(p.pio_addr),
    xlsp(XlsDev::createPeripheral(p, *this)),
    updateEventWrapper([this] { this->updatePeripheralCycle(); }, "Update")
{
    resetPeripheral();
    pioSize = xlsp.getPeripheralSize();
    DPRINTF(XlsDev, "XLS PERIPHERAL: pioSize set to %d\n", pioSize);
}

void
XlsDev::resetPeripheral()
{
    if (updateEventWrapper.scheduled())
        deschedule(updateEventWrapper);
    xlsp.reset();
    updatePeripheralCycle();
}

void
XlsDev::updatePeripheralCycle()
{
    xlsp.update();
    // TODO: Allow specifying update frequency
    schedule(updateEventWrapper, curTick() + updateDivisor * clockPeriod());
}

AddrRangeList
XlsDev::getAddrRanges() const
{
    return AddrRangeList({RangeSize(pioAddr, pioSize)});
}

Tick
XlsDev::eatUpAccessCycle() {
    fatal_if(!updateEventWrapper.scheduled(), "XLS device is not clocked!");

    Tick response_time = updateEventWrapper.when() - curTick();
    reschedule(updateEventWrapper,
               updateEventWrapper.when() + updateDivisor * clockPeriod());

    return response_time;
}

Tick
XlsDev::read(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    xlsp.read(pkt->getPtr<char*>(), daddr, pkt->getSize());

    pkt->makeAtomicResponse();
    return eatUpAccessCycle();
}

Tick
XlsDev::write(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    xlsp.write(pkt->getPtr<char*>(), daddr, pkt->getSize());

    pkt->makeAtomicResponse();
    return eatUpAccessCycle();
}

void
XlsDev::serialize(CheckpointOut &cp) const
{
    warn("XLS peripherals are not serializable. The snapshot won't include "
         "this peripheral's state.\n");
}

void
XlsDev::unserialize(CheckpointIn &cp)
{
    warn("XLS peripheral's state was not recovered - device is not "
         "unserializable.\n");
}

}

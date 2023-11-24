/*
 * Copyright (c) 2023 Antmicro Ltd.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __DEV_COSIM_XLS_HH__
#define __DEV_COSIM_XLS_HH__

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "dev/dma_device.hh"
#include "dev/xls_platform.hh"
#include "params/XlsDev.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5 {

class XlsLibSyms;

struct Xls_C_ErrorOps
{
    void *ctx;
    void (__attribute__((sysv_abi)) *reportError)
        (void *, uint8_t severity, const char *message);
};

struct C_OnRequestCompletionCB
{
    // An argument to be passed to `complete()`
    void* ctx;
    // Called with `ctx` on completion of a memory transfer
    // This might get caled from outside the XLS code.
    void(__attribute__((sysv_abi)) * complete)(void* ctx);
};

class XlsStream
{
  private:
    XlsStream(void *handle_) : handle(handle_) {}

    void *handle = nullptr;

    friend class XlsLibSyms;
    friend class XlsPeripheral;
    friend class XlsPeripheralBuilder;
  public:
};

class XlsPeripheral
{
  private:
    XlsPeripheral(void *handle, DmaDevice &dma_dev, XlsPlatform &platform,
                  uint32_t device_id);

    void *handle = nullptr;
    DmaDevice &dma_dev;
    XlsPlatform &platform;
    uint32_t device_id;

    bool gotError;
    std::vector<std::string> errors;
    Xls_C_ErrorOps error_ops;

    std::string dev_name;

    class XferEvent : public Event
    {
      public:
        XferEvent(XlsPeripheral &xlsp, C_OnRequestCompletionCB response_ops,
                  Addr at);

        ~XferEvent() override = default;

        XlsPeripheral &xlsp;
        C_OnRequestCompletionCB response_ops;
        bool completed = false;
        Addr at;

        void process() override;
    };

    void log(size_t severity, const char *message) const;
    void interrupt(bool state);

    template <typename T, typename F>
    T xlsTry(F procedure);

    friend class XlsLibSyms;

  public:
    operator bool()
    {
        return handle != nullptr;
    }

    void reset();

    template <typename F>
    void forEachChannel(F action);

    size_t getPeripheralSize();

    void update();
    int read(void *data, size_t addr, size_t bytes);
    int write(const void *data, size_t addr, size_t bytes);

    std::string name() const {
      return dma_dev.name();
    }

    ~XlsPeripheral();
};

class XlsDev : public DmaDevice
{
  private:
    Addr pioAddr;
    XlsPeripheral xlsp;
    size_t pioSize;

    EventFunctionWrapper updateEventWrapper;

    void resetPeripheral();
    void updatePeripheralCycle();

    // Reschedules an update cycle of the XLS device.
    Tick eatUpAccessCycle();

    static XlsPeripheral createPeripheral(const XlsDevParams &p,
                                          DmaDevice &dma_dev);

  protected:
    AddrRangeList getAddrRanges() const override;
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  public:
    using Params = XlsDevParams;

    XlsDev(const Params &p);
    ~XlsDev() override = default;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    static const Tick updateDivisor = 10000;
};

}

#endif // __DEV_COSIM_XLS_HH__

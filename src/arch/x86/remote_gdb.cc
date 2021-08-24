/*
 * Copyright 2015 LabWare
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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

#include "arch/x86/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/x86/mmu.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/process.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAcc.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"
#include "sim/workload.hh"

namespace gem5
{

using namespace X86ISA;

RemoteGDB::RemoteGDB(System *_system, int _port) :
    BaseRemoteGDB(_system, _port), regCache32(this), regCache64(this)
{}

bool
RemoteGDB::acc(Addr va, size_t len)
{
    if (FullSystem) {
        Walker *walker = dynamic_cast<MMU *>(
            context()->getMMUPtr())->getDataWalker();
        unsigned logBytes;
        Fault fault = walker->startFunctional(context(), va, logBytes,
                                              BaseMMU::Read);
        if (fault != NoFault)
            return false;

        Addr endVa = va + len - 1;
        if ((va & ~mask(logBytes)) == (endVa & ~mask(logBytes)))
            return true;

        fault = walker->startFunctional(context(), endVa, logBytes,
                                        BaseMMU::Read);
        return fault == NoFault;
    } else {
        return context()->getProcessPtr()->pTable->lookup(va) != nullptr;
    }
}

BaseGdbRegCache*
RemoteGDB::gdbRegs()
{
    // First, try to figure out which type of register cache to return based
    // on the architecture reported by the workload.
    if (system()->workload) {
        auto arch = system()->workload->getArch();
        if (arch == loader::X86_64) {
            return &regCache64;
        } else if (arch == loader::I386) {
            return &regCache32;
        } else if (arch != loader::UnknownArch) {
            panic("Unrecognized workload arch %s.",
                    loader::archToString(arch));
        }
    }

    // If that didn't work, decide based on the current mode of the context.
    HandyM5Reg m5reg = context()->readMiscRegNoEffect(MISCREG_M5_REG);
    if (m5reg.submode == SixtyFourBitMode)
        return &regCache64;
    else
        return &regCache32;
}



void
RemoteGDB::AMD64GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    r.rax = context->readIntReg(int_reg::Rax);
    r.rbx = context->readIntReg(int_reg::Rbx);
    r.rcx = context->readIntReg(int_reg::Rcx);
    r.rdx = context->readIntReg(int_reg::Rdx);
    r.rsi = context->readIntReg(int_reg::Rsi);
    r.rdi = context->readIntReg(int_reg::Rdi);
    r.rbp = context->readIntReg(int_reg::Rbp);
    r.rsp = context->readIntReg(int_reg::Rsp);
    r.r8 = context->readIntReg(int_reg::R8);
    r.r9 = context->readIntReg(int_reg::R9);
    r.r10 = context->readIntReg(int_reg::R10);
    r.r11 = context->readIntReg(int_reg::R11);
    r.r12 = context->readIntReg(int_reg::R12);
    r.r13 = context->readIntReg(int_reg::R13);
    r.r14 = context->readIntReg(int_reg::R14);
    r.r15 = context->readIntReg(int_reg::R15);
    r.rip = context->pcState().pc();
    r.eflags = context->readMiscRegNoEffect(MISCREG_RFLAGS);
    r.cs = context->readMiscRegNoEffect(MISCREG_CS);
    r.ss = context->readMiscRegNoEffect(MISCREG_SS);
    r.ds = context->readMiscRegNoEffect(MISCREG_DS);
    r.es = context->readMiscRegNoEffect(MISCREG_ES);
    r.fs = context->readMiscRegNoEffect(MISCREG_FS);
    r.gs = context->readMiscRegNoEffect(MISCREG_GS);
}

void
RemoteGDB::X86GdbRegCache::getRegs(ThreadContext *context)
{
    DPRINTF(GDBAcc, "getRegs in remotegdb \n");
    r.eax = context->readIntReg(int_reg::Rax);
    r.ecx = context->readIntReg(int_reg::Rcx);
    r.edx = context->readIntReg(int_reg::Rdx);
    r.ebx = context->readIntReg(int_reg::Rbx);
    r.esp = context->readIntReg(int_reg::Rsp);
    r.ebp = context->readIntReg(int_reg::Rbp);
    r.esi = context->readIntReg(int_reg::Rsi);
    r.edi = context->readIntReg(int_reg::Rdi);
    r.eip = context->pcState().pc();
    r.eflags = context->readMiscRegNoEffect(MISCREG_RFLAGS);
    r.cs = context->readMiscRegNoEffect(MISCREG_CS);
    r.ss = context->readMiscRegNoEffect(MISCREG_SS);
    r.ds = context->readMiscRegNoEffect(MISCREG_DS);
    r.es = context->readMiscRegNoEffect(MISCREG_ES);
    r.fs = context->readMiscRegNoEffect(MISCREG_FS);
    r.gs = context->readMiscRegNoEffect(MISCREG_GS);
}

void
RemoteGDB::AMD64GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");
    context->setIntReg(int_reg::Rax, r.rax);
    context->setIntReg(int_reg::Rbx, r.rbx);
    context->setIntReg(int_reg::Rcx, r.rcx);
    context->setIntReg(int_reg::Rdx, r.rdx);
    context->setIntReg(int_reg::Rsi, r.rsi);
    context->setIntReg(int_reg::Rdi, r.rdi);
    context->setIntReg(int_reg::Rbp, r.rbp);
    context->setIntReg(int_reg::Rsp, r.rsp);
    context->setIntReg(int_reg::R8, r.r8);
    context->setIntReg(int_reg::R9, r.r9);
    context->setIntReg(int_reg::R10, r.r10);
    context->setIntReg(int_reg::R11, r.r11);
    context->setIntReg(int_reg::R12, r.r12);
    context->setIntReg(int_reg::R13, r.r13);
    context->setIntReg(int_reg::R14, r.r14);
    context->setIntReg(int_reg::R15, r.r15);
    context->pcState(r.rip);
    context->setMiscReg(MISCREG_RFLAGS, r.eflags);
    if (r.cs != context->readMiscRegNoEffect(MISCREG_CS))
        warn("Remote gdb: Ignoring update to CS.\n");
    if (r.ss != context->readMiscRegNoEffect(MISCREG_SS))
        warn("Remote gdb: Ignoring update to SS.\n");
    if (r.ds != context->readMiscRegNoEffect(MISCREG_DS))
        warn("Remote gdb: Ignoring update to DS.\n");
    if (r.es != context->readMiscRegNoEffect(MISCREG_ES))
        warn("Remote gdb: Ignoring update to ES.\n");
    if (r.fs != context->readMiscRegNoEffect(MISCREG_FS))
        warn("Remote gdb: Ignoring update to FS.\n");
    if (r.gs != context->readMiscRegNoEffect(MISCREG_GS))
        warn("Remote gdb: Ignoring update to GS.\n");
}

void
RemoteGDB::X86GdbRegCache::setRegs(ThreadContext *context) const
{
    DPRINTF(GDBAcc, "setRegs in remotegdb \n");
    context->setIntReg(int_reg::Rax, r.eax);
    context->setIntReg(int_reg::Rcx, r.ecx);
    context->setIntReg(int_reg::Rdx, r.edx);
    context->setIntReg(int_reg::Rbx, r.ebx);
    context->setIntReg(int_reg::Rsp, r.esp);
    context->setIntReg(int_reg::Rbp, r.ebp);
    context->setIntReg(int_reg::Rsi, r.esi);
    context->setIntReg(int_reg::Rdi, r.edi);
    context->pcState(r.eip);
    context->setMiscReg(MISCREG_RFLAGS, r.eflags);
    if (r.cs != context->readMiscRegNoEffect(MISCREG_CS))
        warn("Remote gdb: Ignoring update to CS.\n");
    if (r.ss != context->readMiscRegNoEffect(MISCREG_SS))
        warn("Remote gdb: Ignoring update to SS.\n");
    if (r.ds != context->readMiscRegNoEffect(MISCREG_DS))
        warn("Remote gdb: Ignoring update to DS.\n");
    if (r.es != context->readMiscRegNoEffect(MISCREG_ES))
        warn("Remote gdb: Ignoring update to ES.\n");
    if (r.fs != context->readMiscRegNoEffect(MISCREG_FS))
        warn("Remote gdb: Ignoring update to FS.\n");
    if (r.gs != context->readMiscRegNoEffect(MISCREG_GS))
        warn("Remote gdb: Ignoring update to GS.\n");
}

} // namespace gem5

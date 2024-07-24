/*
 * Copyright (c) 2015, University of Kaiserslautern
 * Copyright (c) 2016, Dresden University of Technology (TU Dresden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "systemc/tlm_bridge/sc_ext.hh"

#include <optional>

#include "systemc/ext/utils/sc_report_handler.hh"
#include "systemc/tlm_bridge/gem5_to_tlm.hh"
#include "systemc/tlm_bridge/tlm_to_gem5.hh"

using namespace gem5;

namespace Gem5SystemC
{
namespace
{

struct ControlConversionRegister
{
    ControlConversionRegister()
    {
        sc_gem5::addPayloadToPacketConversionStep(
            [] (PacketPtr pkt, tlm::tlm_generic_payload &trans)
            {
                ControlExtension *control_ex = nullptr;
                trans.get_extension(control_ex);
                if (!control_ex) {
                    return;
                }

                if (control_ex->isPrivileged()) {
                    pkt->req->setFlags(Request::PRIVILEGED);
                } else {
                    pkt->req->clearFlags(Request::PRIVILEGED);
                }

                if (control_ex->isSecure()) {
                    pkt->req->setFlags(Request::SECURE);
                } else {
                    pkt->req->clearFlags(Request::SECURE);
                }

                if (control_ex->isInstruction()) {
                    pkt->req->setFlags(Request::INST_FETCH);
                } else {
                    pkt->req->clearFlags(Request::INST_FETCH);
                }

                pkt->qosValue(control_ex->getQos());

                if (control_ex->hasStreamId()) {
                    pkt->req->setStreamId(control_ex->getStreamId().value());
                }
                if (control_ex->hasSubstreamId()) {
                    pkt->req->setSubstreamId(
                        control_ex->getSubstreamId().value());
                }
            });
        sc_gem5::addPacketToPayloadConversionStep(
            [] (PacketPtr pkt, tlm::tlm_generic_payload &trans)
            {
                ControlExtension *control_ex = nullptr;
                trans.get_extension(control_ex);
                if (!control_ex) {
                    control_ex = new Gem5SystemC::ControlExtension();
                    if (trans.has_mm()) {
                      trans.set_auto_extension(control_ex);
                    } else {
                      trans.set_extension(control_ex);
                    }
                }

                control_ex->setPrivileged(pkt->req->isPriv());
                control_ex->setSecure(pkt->req->isSecure());
                control_ex->setInstruction(pkt->req->isInstFetch());
                control_ex->setQos(pkt->qosValue());
                if (pkt->req->hasStreamId()) {
                    control_ex->setStreamId(pkt->req->streamId());
                }
                if (pkt->req->hasSubstreamId()) {
                    control_ex->setSubstreamId(pkt->req->substreamId());
                }
            });
    }
};

} // namespace

Gem5Extension::Gem5Extension(PacketPtr p) : packet(p)
{
}

Gem5Extension &
Gem5Extension::getExtension(const tlm::tlm_generic_payload *payload)
{
    Gem5Extension *result = nullptr;
    payload->get_extension(result);
    sc_assert(result != nullptr);
    return *result;
}

Gem5Extension &
Gem5Extension::getExtension(const tlm::tlm_generic_payload &payload)
{
    return Gem5Extension::getExtension(&payload);
}

PacketPtr
Gem5Extension::getPacket()
{
    return packet;
}

tlm::tlm_extension_base *
Gem5Extension::clone() const
{
    return new Gem5Extension(packet);
}

void
Gem5Extension::copy_from(const tlm::tlm_extension_base &ext)
{
    const Gem5Extension &from = static_cast<const Gem5Extension &>(ext);
    packet = from.packet;
}

AtomicExtension::AtomicExtension(
    std::shared_ptr<gem5::AtomicOpFunctor> o, bool r)
    : op(o), returnRequired(r)
{
}

tlm::tlm_extension_base *
AtomicExtension::clone() const
{
    return new AtomicExtension(*this);
}

void
AtomicExtension::copy_from(const tlm::tlm_extension_base &ext)
{
    const AtomicExtension &from = static_cast<const AtomicExtension &>(ext);
    *this = from;
}

AtomicExtension &
AtomicExtension::getExtension(const tlm::tlm_generic_payload &payload)
{
    return AtomicExtension::getExtension(&payload);
}

AtomicExtension &
AtomicExtension::getExtension(const tlm::tlm_generic_payload *payload)
{
    AtomicExtension *result = nullptr;
    payload->get_extension(result);
    sc_assert(result);
    return *result;
}

bool
AtomicExtension::isReturnRequired() const
{
    return returnRequired;
}

gem5::AtomicOpFunctor*
AtomicExtension::getAtomicOpFunctor() const
{
    return op.get();
}

ControlExtension::ControlExtension()
    : privileged(false), secure(false), instruction(false), qos(0)
{
    [[maybe_unused]] static ControlConversionRegister *conversion_register =
        new ControlConversionRegister();
}

tlm::tlm_extension_base *
ControlExtension::clone() const
{
    return new ControlExtension(*this);
}

void
ControlExtension::copy_from(const tlm::tlm_extension_base &ext)
{
    const ControlExtension &from = static_cast<const ControlExtension &>(ext);
    *this = from;
}

ControlExtension &
ControlExtension::getExtension(const tlm::tlm_generic_payload &payload)
{
    return ControlExtension::getExtension(&payload);
}

ControlExtension &
ControlExtension::getExtension(const tlm::tlm_generic_payload *payload)
{
    ControlExtension *result = nullptr;
    payload->get_extension(result);
    sc_assert(result);
    return *result;
}

bool
ControlExtension::isPrivileged() const
{
    return privileged;
}

void
ControlExtension::setPrivileged(bool p)
{
    privileged = p;
}

bool
ControlExtension::isSecure() const
{
    return secure;
}

void
ControlExtension::setSecure(bool s)
{
    secure = s;
}

bool
ControlExtension::isInstruction() const
{
    return instruction;
}

void
ControlExtension::setInstruction(bool i)
{
    instruction = i;
}

uint8_t
ControlExtension::getQos() const
{
    return qos;
}

void
ControlExtension::setQos(uint8_t q)
{
    qos = q;
}

bool
ControlExtension::hasStreamId() const
{
    return stream_id.has_value();
}

std::optional<uint32_t>
ControlExtension::getStreamId() const
{
    return stream_id;
}

void
ControlExtension::setStreamId(std::optional<uint32_t> s)
{
    stream_id = std::move(s);
}

bool
ControlExtension::hasSubstreamId() const
{
    return substream_id.has_value();
}

std::optional<uint32_t>
ControlExtension::getSubstreamId() const
{
    return substream_id;
}

void
ControlExtension::setSubstreamId(std::optional<uint32_t> s)
{
    substream_id = std::move(s);
}

bool
ControlExtension::hasBufferable() const
{
    return bufferable.has_value();
}

std::optional<bool>
ControlExtension::getBufferable() const
{
    return bufferable;
}

void
ControlExtension::setBufferable(std::optional<bool> b)
{
    bufferable = std::move(b);
}

bool
ControlExtension::hasModifiable() const
{
    return modifiable.has_value();
}

std::optional<bool>
ControlExtension::getModifiable() const
{
    return modifiable;
}

void
ControlExtension::setModifiable(std::optional<bool> m)
{
    modifiable = std::move(m);
}

bool
ControlExtension::hasReadAllocate() const
{
    return read_allocate.has_value();
}

std::optional<bool>
ControlExtension::getReadAllocate() const
{
    return read_allocate;
}

void
ControlExtension::setReadAllocate(std::optional<bool> ra)
{
    read_allocate = std::move(ra);
}

bool
ControlExtension::hasWriteAllocate() const
{
    return write_allocate.has_value();
}

std::optional<bool>
ControlExtension::getWriteAllocate() const
{
    return write_allocate;
}

void
ControlExtension::setWriteAllocate(std::optional<bool> wa)
{
    write_allocate = std::move(wa);
}

void
ControlExtension::setAXI4CacheAttributesFromFlag(uint32_t axi4_flag)
{
    setBufferable((axi4_flag >> 0) & 1);
    setModifiable((axi4_flag >> 1) & 1);
    setReadAllocate((axi4_flag >> 2) & 1);
    setWriteAllocate((axi4_flag >> 3) & 1);
}

bool
ControlExtension::hasDomain() const
{
    return domain.has_value();
}

std::optional<uint32_t>
ControlExtension::getDomain() const
{
    return domain;
}

void
ControlExtension::setDomain(std::optional<uint32_t> d)
{
    domain = std::move(d);
}

}  // namespace Gem5SystemC

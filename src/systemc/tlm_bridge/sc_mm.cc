/*
 * Copyright (c) 2015, University of Kaiserslautern
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

#include "systemc/tlm_bridge/sc_mm.hh"

namespace Gem5SystemC
{

MemoryManager::MemoryManager() : numberOfAllocations(0), numberOfFrees(0) {}

MemoryManager::~MemoryManager()
{
    for (gp *payload: freePayloads) {
        delete payload;
        numberOfFrees++;
    }
}

gp *
MemoryManager::allocate()
{
    if (freePayloads.empty()) {
        numberOfAllocations++;
        return new gp(this);
    } else {
        gp *result = freePayloads.back();
        freePayloads.pop_back();
        return result;
    }
}

void
MemoryManager::free(gp *payload)
{
    payload->reset(); // clears all extensions
    payload->set_address(0);
    payload->set_command(tlm::TLM_IGNORE_COMMAND);
    payload->set_data_ptr(0);
    payload->set_data_length(0);
    payload->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    payload->set_dmi_allowed(false);
    payload->set_byte_enable_ptr(0);
    payload->set_byte_enable_length(0);
    payload->set_streaming_width(0);
    freePayloads.push_back(payload);
}

} // namespace Gem5SystemC

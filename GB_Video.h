// Copyright (c) 2017, Jean Charles Vallieres
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <array>
#include <memory>

#include "GB.h"
#include "GB_Types.h"

namespace GB {

class Video {
public:

    struct FrameBuf {
        std::array<u32, 160 * 144> pix = {};
    };

    std::array<u8, 8 * 1024> VRAM = {};
    std::array<u8, 256> OAM = {};

    union {
        struct {
            u8 bgdisplay : 1;
            u8 objdisplay : 1;
            u8 objsize : 1;
            u8 bgtilemap : 1;
            u8 bgwintiledata : 1;
            u8 windisp : 1;
            u8 wintilemap : 1;
            u8 lcden : 1;
        } bits; u8 value = 0;
    } LCDC = {};

    union {
        struct {
            u8 mode : 2;
            u8 coincidence : 1;
            u8 mode0int : 1;
            u8 mode1int : 1;
            u8 mode2int : 1;
            u8 coincidenceint : 1;
        } bits; u8 value = 0;
    } STAT = {};

    u8 SCY = 0;
    u8 SCX = 0;
    u8 LY = 0;
    u8 LYC = 0;
    u8 BGP = 0;
    u8 OBP0 = 0;
    u8 OBP1 = 0;
    u8 WY = 0;
    u8 WX = 0;

    u8 LCDX = 0;

    u8 m_mode = 0;

    std::unique_ptr<FrameBuf> m_upFront_buf;
    std::unique_ptr<FrameBuf> m_upBack_buf;

    void Reset(ResetOption reset_opt);

    void Tick(System& rSystem);

    void Flip(System& rSystem);
    void RenderLine();

    template<Access eAccess> u8 VRAMAccess(const u16 addr, const u8 v = 0);
    template<Access eAccess> u8 OAMAccess(const u8 addr, const u8 v = 0);

    void UpdateSTAT() { 
        STAT.bits.coincidence = (LY == LYC) ? 1 : 0;
        STAT.bits.mode = m_mode;
    }
};

} // namespace GB
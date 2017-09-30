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

#include "GB.h"
#include "GB_Types.h"

namespace GB {

class IO {
public:
    u8 P1 = 0;
    u8 SB = 0;
    u8 SC = 0;
    union {
        struct DividerReg {
            u8 low;
            u8 DIV;
        } details;
        u16 value = 0;
    } Divider = {};

    u8 TIMA = 0;
    u8 TMA  = 0;
    u8 TAC  = 0;

    u8 m_timer_mask = 255;

    bool m_serial_active = false;
    bool m_serial_clock  = false;
    u8   m_serial_bits   = 0;

    Keys m_keys = {};

    void Reset(ResetOption reset_opt);

    void Tick(System& m_rSystem);

    u8   MakeP1(System& rSystem);
    void SetTAC(u8 v);

    template<Access eAccess>
    u8 RegAccess(System& rSystem, const u8 addr, const u8 v = 0);
};

} // namespace GB

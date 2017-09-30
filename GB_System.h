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
#include <chrono>

#include "GB.h"
#include "GB_Types.h"
#include "GB_CPU.h"
#include "GB_Video.h"
#include "GB_GamePak.h"
#include "GB_IO.h"
#include "GB_Audio.h"

namespace GB {

class System {
public:
    CPU m_cpu = {};

    bool m_new_frame    = false;
    u32  m_frame_number = 0;
    u64  m_cycle_count  = 0;

    Video   m_video    = {};
    GamePak m_game_pak = {};
    IO      m_io       = {};
    Audio   m_audio    = {};

    std::array<u8, 8 * 1024> RAM = {};

    std::array<u8, 256> m_boot_ROM = {};

    bool m_boot_rom_active = false;

    void Tick();

    void Reset(ResetOption reset_opt);
    void RunFrame();

    std::chrono::nanoseconds RunTime(const std::chrono::nanoseconds& time_to_run);

    template<Access eAccess>
    u8 BusAccess(u16 addr, u8 v = 0);
};

} // namespace GB

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

namespace GB {

void System::Tick()
{
    m_video.Tick(*this);
    m_io.Tick(*this);
    m_audio.Tick(*this);
    m_cpu.DoOAMDMA(*this);
    ++m_cycle_count;
}

void System::Reset(ResetOption reset_opt)
{
    m_boot_rom_active = (reset_opt & ResetOption_USE_BOOT_ROM) != 0;
    m_cpu.Reset(reset_opt);
    m_video.Reset(reset_opt);
    m_io.Reset(reset_opt);
    m_audio.Reset(reset_opt);
    m_frame_number = 0;
    m_cycle_count  = 0;
    m_new_frame    = false;
}

void System::RunFrame()
{
    m_new_frame = false;
    while (!m_new_frame) {
        m_cpu.Execute(*this);
    }
}

nanoseconds System::RunTime(const nanoseconds& time_to_run)
{
    const u64 cycles_to_run     = time_to_run.count() * 1024LL * 1024LL / 1000000000LL;
    const u64 start_cycle_count = m_cycle_count;
    const u64 end_cycle_count   = start_cycle_count + cycles_to_run;

    while (m_cycle_count < end_cycle_count) {
        m_cpu.Execute(*this);
    }

    return nanoseconds((m_cycle_count - start_cycle_count) * 1000000000LL / (1024LL * 1024LL));
}

template<Access eAccess>
u8 System::BusAccess(u16 addr, u8 v)
{
    if (addr < 0x100 && m_boot_rom_active) {
        return m_boot_ROM[addr];
    }
    if (addr < 0x8000) {
        return m_game_pak.ROMAccess<eAccess>(addr, v);
    } else if (addr < 0xa000) {
        return m_video.VRAMAccess<eAccess>(addr, v);
    } else if (addr < 0xc000) {
        return m_game_pak.RAMAccess<eAccess>(addr, v);
    } else if (addr < 0xfe00) {
        if constexpr (eAccess == Access::Write) {
            RAM[addr & 0x1fff] = v;
        } else {
            return RAM[addr & 0x1fff];
        }
    } else if (addr < 0xff00) {
        return m_video.OAMAccess<eAccess>(addr & 0xff, v);
    } else if (addr < 0xff80) {
        return m_io.RegAccess<eAccess>(*this, addr & 0xff, v);
    } else {
        if constexpr (eAccess == Access::Write) {
            m_cpu.HRAM[addr & 0x7f] = v;
        } else {
            return m_cpu.HRAM[addr & 0x7f];
        }
    }

    if constexpr (eAccess == Access::Write) {
        return 0;
    }
}

} // namespace GB

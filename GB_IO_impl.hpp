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

#define STD_REG(x,y) case x : if (eAccess == Access::Write) {y = v;} else {return y;} break;

template<Access eAccess>
u8 IO::RegAccess(System& rSystem, const u8 addr, const u8 v)
{
#pragma warning( push )
#pragma warning( disable : 4127 )   // warning C4127: conditional expression is constant

    if (addr >= 0x10 && addr <= 0x3F)
    {
        return rSystem.m_audio.RegAccess<eAccess>(addr, v);
    }

    switch (addr)
    {
    case 0x00 : 
        if (eAccess == Access::Write) {
            P1 = v;
        } else {
            return MakeP1(rSystem);
        } 
        break;
        STD_REG(0x01, SB);
    case 0x02 :
        if (eAccess == Access::Write) {
            m_serial_clock = (v & 1) != 0;
            m_serial_active = (v & 0x80) != 0;
            if (m_serial_active) {
                m_serial_bits = 8;
            }
        } else {
            return (m_serial_clock ? 1 : 0) | (m_serial_active ? 0x80 : 0) ;
        }
        break;
    case 0x04 :
        if (eAccess == Access::Write) {
            Divider.value = 0;
        } else {
            return Divider.details.DIV;
        } 
        break;
    case 0x05 : 
        if (eAccess == Access::Write) {
            TIMA = v;
        } else {
            return TIMA;
        } 
        break;
        STD_REG(0x06, TMA);
    case 0x07 :
        if (eAccess == Access::Write) {
            SetTAC(v);
        } else {
            return TAC;}
        break;
        STD_REG(0x0f, rSystem.m_cpu.IF.value);
    case 0x40 : 
        if (eAccess == Access::Write) {
            if (!rSystem.m_video.LCDC.bits.lcden && (v & 0x80)) {
                // Restart video
                rSystem.m_video.m_mode = 1;
                rSystem.m_video.LY = 153;
                rSystem.m_video.LCDX = 113;
            } else if (rSystem.m_video.LCDC.bits.lcden && ((v & 0x80) == 0)) {
                rSystem.m_video.m_mode = 1;
            }
            rSystem.m_video.LCDC.value = v;
        } else {
            return rSystem.m_video.LCDC.value;
        }
        break;
    case 0x41 : 
        if (eAccess == Access::Write) {
            rSystem.m_video.STAT.value = v & 0x78;
        } else {
            rSystem.m_video.UpdateSTAT();
            return rSystem.m_video.STAT.value;
        }
        break;
        STD_REG(0x42, rSystem.m_video.SCY);
        STD_REG(0x43, rSystem.m_video.SCX);
    case 0x44 : 
        if (eAccess == Access::Read) {
            return rSystem.m_video.LY;
        }
        // Writes are ignored
        break;
        STD_REG(0x45, rSystem.m_video.LYC);
    case 0x46 : 
        if (eAccess == Access::Write) {
            rSystem.m_cpu.m_oam_dma_active = true;
            rSystem.m_cpu.m_oam_dma_src = v << 8;
            rSystem.m_cpu.m_oam_dma_index = 0;
        } 
        break;
        STD_REG(0x47, rSystem.m_video.BGP);
        STD_REG(0x48, rSystem.m_video.OBP0);
        STD_REG(0x49, rSystem.m_video.OBP1);
        STD_REG(0x4A, rSystem.m_video.WY);
        STD_REG(0x4B, rSystem.m_video.WX);
    case 0x50:
        if (eAccess == Access::Write) {
            if (v & 1) {
                rSystem.m_boot_rom_active = false;
            }
        } else {
            return 0;
        }
        break;
    }

    return 0;

#pragma warning( pop )
}

void IO::Reset(ResetOption)
{
    Divider.value = 0;
    TIMA = 0;
    TMA = 0;
    TAC = 0xF8;
    m_timer_mask = 255;
    P1 = 0x0f;
    m_keys.value = 0xff;
    m_serial_active = false;
    m_serial_bits = 0;
    m_serial_clock = false;
}

void IO::Tick(System& rSystem)
{
    Divider.value += 4;

    if (TAC & 4) {
        if (((u8)rSystem.m_cycle_count & m_timer_mask) == 0) {
            if (TIMA == 255) {
                TIMA = TMA;
                rSystem.m_cpu.IF.f.timer = 1;
            } else {
                ++TIMA;
            }
        }
    }

    if ((rSystem.m_cycle_count & 127) == 0) {
        if (m_serial_active && m_serial_clock && m_serial_bits) {
            --m_serial_bits;
            SB = (SB << 1) | 1;
            if (!m_serial_bits) {
                m_serial_active = false;
                rSystem.m_cpu.IF.f.serial = 1;
            }
        }
    }
}

u8 IO::MakeP1(System& rSystem)
{
    const u8 prev_P1 = P1;
    u8 keys_val = 0xf;
    if (P1 & 0x20) {keys_val = keys_val & m_keys.value;}
    if (P1 & 0x10) {keys_val = keys_val & (m_keys.value >> 4);}
    P1 = (P1 & 0x30) | keys_val | 0xc0;
    if (prev_P1 & (~P1) & 0xf)
    {
        rSystem.m_cpu.IF.f.joypad = 1;
    }
    return P1;
}

void IO::SetTAC(u8 val)
{
    TAC = val;
    switch (val & 3) {
    case 0 : m_timer_mask = 255; break;
    case 1 : m_timer_mask = 3; break;
    case 2 : m_timer_mask = 15; break;
    case 3 : m_timer_mask = 63; break;
    }
}

} // namespace GB

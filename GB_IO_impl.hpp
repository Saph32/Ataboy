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

#define STD_REG(y)                                                                                           \
    if constexpr (eAccess == Access::Write) {                                                                          \
        y = v;                                                                                               \
    } else {                                                                                                 \
        return y;                                                                                            \
    }                                                                                                        \
    break;

template<Access eAccess>
u8 IO::RegAccess(System& rSystem, const u8 addr, const u8 v)
{
    if (addr >= 0x10 && addr <= 0x3F) {
        return rSystem.m_audio.RegAccess<eAccess>(addr, v);
    }

    switch (addr) {
    case 0x00:
        if constexpr (eAccess == Access::Write) {
            P1.detail.dpad    = (v & 0x10) ? 1 : 0;
            P1.detail.buttons = (v & 0x20) ? 1 : 0;
        } else {
            return MakeP1(rSystem);
        }
        break;
    case 0x01: STD_REG(SB);
    case 0x02:
        if constexpr (eAccess == Access::Write) {
            m_serial_clock  = (v & 1) != 0;
            m_serial_active = (v & 0x80) != 0;
            if (m_serial_active) {
                m_serial_bits = 8;
            }
        } else {
            return (m_serial_clock ? 1 : 0) | (m_serial_active ? 0x80 : 0);
        }
        break;
    case 0x04:
        if constexpr (eAccess == Access::Write) {
            Divider.value = 0;
        } else {
            return Divider.details.DIV;
        }
        break;
    case 0x05:
        if constexpr (eAccess == Access::Write) {
            TIMA = v;
        } else {
            return TIMA;
        }
        break;
    case 0x06: STD_REG(TMA);
    case 0x07:
        if constexpr (eAccess == Access::Write) {
            SetTAC(v);
        } else {
            return TAC;
        }
        break;
    case 0x0f: STD_REG(rSystem.m_cpu.IF.value);
    case 0x40:
        if constexpr (eAccess == Access::Write) {
            if (!rSystem.m_video.LCDC.bits.lcden && (v & 0x80)) {
                // Restart video
                rSystem.m_video.m_mode = 1;
                rSystem.m_video.LY     = 153;
                rSystem.m_video.LCDX   = 113;
            } else if (rSystem.m_video.LCDC.bits.lcden && ((v & 0x80) == 0)) {
                rSystem.m_video.m_mode = 1;
            }
            rSystem.m_video.LCDC.value = v;
        } else {
            return rSystem.m_video.LCDC.value;
        }
        break;
    case 0x41:
        if constexpr (eAccess == Access::Write) {
            rSystem.m_video.STAT.value = v & 0x78;

            // Weird DMG bug. When writting to STAT in blank mode, an interrupt is risen.
            // source : http://www.devrs.com/gb/files/faqs.html#GBBugs
            // This is required by Zerd no Densetsu. That game actually doesn't work in a CGB.
            if (rSystem.m_video.LCDC.bits.lcden) {
                if (rSystem.m_video.m_mode == 1 || rSystem.m_video.m_mode == 0) {
                    rSystem.m_cpu.IF.f.lcdstat = 1;
                }
            }
        } else {
            rSystem.m_video.UpdateSTAT();
            return rSystem.m_video.STAT.value;
        }
        break;
    case 0x42: STD_REG(rSystem.m_video.SCY);
    case 0x43: STD_REG(rSystem.m_video.SCX);
    case 0x44:
        if constexpr (eAccess == Access::Read) {
            return rSystem.m_video.LY;
        }
        // Writes are ignored
        break;
    case 0x45: STD_REG(rSystem.m_video.LYC);
    case 0x46:
        if constexpr (eAccess == Access::Write) {
            rSystem.m_cpu.m_oam_dma_active = true;
            rSystem.m_cpu.m_oam_dma_src    = v << 8;
            rSystem.m_cpu.m_oam_dma_index  = 0;
        }
        break;
    case 0x47: STD_REG(rSystem.m_video.BGP);
    case 0x48: STD_REG(rSystem.m_video.OBP0);
    case 0x49: STD_REG(rSystem.m_video.OBP1);
    case 0x4A: STD_REG(rSystem.m_video.WY);
    case 0x4B: STD_REG(rSystem.m_video.WX);
    case 0x50:
        if constexpr (eAccess == Access::Write) {
            if (v & 1) {
                rSystem.m_boot_rom_active = false;
            }
        } else {
            return 0;
        }
        break;
    }

    return 0;
}

void IO::Reset(ResetOption)
{
    Divider.value   = 0;
    TIMA            = 0;
    TMA             = 0;
    TAC             = 0xF8;
    m_timer_mask    = 255;
    P1.value        = 0xff;
    m_keys.value    = 0x00;
    m_serial_active = false;
    m_serial_bits   = 0;
    m_serial_clock  = false;
}

void IO::Tick(System& rSystem)
{
    Divider.value += 4;

    if (TAC & 4) {
        if (((u8)rSystem.m_cycle_count & m_timer_mask) == 0) {
            if (TIMA == 255) {
                TIMA                     = TMA;
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
                m_serial_active           = false;
                rSystem.m_cpu.IF.f.serial = 1;
            }
        }
    }
}

u8 IO::MakeP1(System& rSystem)
{
    u8 keys = 0xf;

    if (!P1.detail.dpad) {
        keys = keys & (~m_keys.detail.dpad.value);
    }
    if (!P1.detail.buttons) {
        keys = keys & (~m_keys.detail.buttons.value);
    }

    P1.detail.keys = keys;

    if ((~P1.value) & 0xf) {
        rSystem.m_cpu.IF.f.joypad = 1;
    }
    return P1.value;
}

void IO::SetTAC(u8 val)
{
    TAC = val;
    switch (val & 3) {
    case 0: m_timer_mask = 255; break;
    case 1: m_timer_mask = 3; break;
    case 2: m_timer_mask = 15; break;
    case 3: m_timer_mask = 63; break;
    }
}

} // namespace GB

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

void Audio::Reset(ResetOption reset_opt)
{

    m_sq1.Reset(*this);
    m_sq2.Reset(*this);
    m_wave.Reset(*this);
    m_noise.Reset(*this);

    // Boot rom state
    if ((reset_opt & ResetOption_USE_BOOT_ROM) == 0) {
        RegAccess<Access::Write>(0x26, 0x80);
        RegAccess<Access::Write>(0x10, 0x80);
        RegAccess<Access::Write>(0x11, 0xBF);
        RegAccess<Access::Write>(0x12, 0xF3);
        RegAccess<Access::Write>(0x14, 0xBF);
        RegAccess<Access::Write>(0x16, 0x3F);
        RegAccess<Access::Write>(0x17, 0x00);
        RegAccess<Access::Write>(0x19, 0xBF);
        RegAccess<Access::Write>(0x1A, 0x7F);
        RegAccess<Access::Write>(0x1B, 0xFF);
        RegAccess<Access::Write>(0x1C, 0x9F);
        RegAccess<Access::Write>(0x1E, 0xBF);
        RegAccess<Access::Write>(0x20, 0xFF);
        RegAccess<Access::Write>(0x21, 0x00);
        RegAccess<Access::Write>(0x22, 0x00);
        RegAccess<Access::Write>(0x23, 0xBF);
        RegAccess<Access::Write>(0x24, 0x77);
        RegAccess<Access::Write>(0x25, 0xF3);

        // Disable boot up sound
        m_sq1.m_active   = false;
        m_sq2.m_active   = false;
        m_wave.m_active  = false;
        m_noise.m_active = false;
    }

    m_left  = 0;
    m_right = 0;
}

void Audio::Tick(System&)
{

    if (m_master_enable) {
        // Advance sequencer
        if ((m_audio_clock & 2047) == 0) {
            // Advance lenght
            if ((m_sequencer_clock & 1) == 0) {
                m_sq1.RunLenght();
                m_sq2.RunLenght();
                m_wave.RunLenght();
                m_noise.RunLenght();
            }

            // Advance sweep
            if (m_sequencer_clock == 2 || m_sequencer_clock == 6) {
                m_sq1.RunSweep();
            }

            // Advance enveloppe
            if (m_sequencer_clock == 7) {
                m_sq1.RunEnv(m_sq1.m_env);
                m_sq2.RunEnv(m_sq2.m_env);
                m_noise.RunEnv(m_noise.m_env);
            }

            m_sequencer_clock = (m_sequencer_clock + 1) & 7;
        }

        // Channel 1
        m_sq1.RunSquare(m_sq1.m_sq);

        // Channel 2
        m_sq2.RunSquare(m_sq2.m_sq);

        // Channel 3
        m_wave.RunWave(*this);
        m_wave.RunWave(*this);

        // Channel 4
        m_noise.RunNoise();
    }

    if (m_master_enable) {
        i16 left  = 0;
        i16 right = 0;
        i16 chan1 = (m_sq1.m_active && m_sq1.m_out_val) ? (int)m_sq1.m_env.vol : 0;
        i16 chan2 = (m_sq2.m_active && m_sq2.m_out_val) ? (int)m_sq2.m_env.vol : 0;
        i16 chan3 = m_wave.m_active ? (int)m_wave.m_out_val : 0;
        i16 chan4 = (m_noise.m_active && m_noise.m_out_val) ? (int)m_noise.m_env.vol : 0;

        auto fnApplyDac = [](SoundChannel& rChan, i16 chan_val) -> i16 {
            if (rChan.m_dac_active) {
                return chan_val - 8;
            } else {
                return 0;
            }
        };

        chan1 = fnApplyDac(m_sq1, chan1);
        chan2 = fnApplyDac(m_sq2, chan2);
        chan3 = fnApplyDac(m_wave, chan3);
        chan4 = fnApplyDac(m_noise, chan4);

        if (NR51 & 0x80) {
            left += chan4;
        }
        if (NR51 & 0x40) {
            left += chan3;
        }
        if (NR51 & 0x20) {
            left += chan2;
        }
        if (NR51 & 0x10) {
            left += chan1;
        }
        if (NR51 & 0x8) {
            right += chan4;
        }
        if (NR51 & 0x4) {
            right += chan3;
        }
        if (NR51 & 0x2) {
            right += chan2;
        }
        if (NR51 & 0x1) {
            right += chan1;
        }
        m_left += left * (int)m_left_vol;
        m_right += right * (int)m_right_vol;
    }

    ++m_audio_clock;
    if ((m_audio_clock & 31) == 0) {
        auto& rSample = m_audio_buf[m_audio_pos & AUDIO_BUF_SIZE_MASK];
        ++m_audio_pos;

        m_left *= 2;
        m_right *= 2;

        rSample.left  = m_left - (i16)(m_left_cap / 32768);
        rSample.right = m_right - (i16)(m_right_cap / 32768);

        m_left_cap  = m_left * 176 + m_left_cap * 32592 / 32768;
        m_right_cap = m_right * 176 + m_right_cap * 32592 / 32768;

        m_left  = 0;
        m_right = 0;
    }
}

template<>
u8 Audio::RegAccess<Access::Read>(const u8 addr, const u8)
{

    if (addr >= 0x30 && addr < 0x40) {
        if (m_wave.m_active) {
            return m_wave.m_cur_wave;
        } else {
            return AUD3WAVERAM[addr - 0x30];
        }
    }

    switch (addr) {
    case 0x10: return NR10 | 0x80;
    case 0x11: return NR11 | 0x3f;
    case 0x12: return NR12;
    case 0x13: return 0xff;
    case 0x14: return NR14 | 0xbf;
    case 0x15: return 0xff;
    case 0x16: return NR21 | 0x3f;
    case 0x17: return NR22;
    case 0x18: return 0xff;
    case 0x19: return NR24 | 0xbf;
    case 0x1a: return NR30 | 0x7f;
    case 0x1b: return 0xff;
    case 0x1c: return NR32 | 0x9f;
    case 0x1d: return 0xff;
    case 0x1e: return NR34 | 0xbf;
    case 0x1f: return 0xff;
    case 0x20: return 0xff;
    case 0x21: return NR42;
    case 0x22: return NR43;
    case 0x23: return NR44 | 0xbf;
    case 0x24: return NR50;
    case 0x25: return NR51;
    case 0x26:
        if (m_master_enable) {
            return 0xF0 | (m_sq1.m_active ? 0x01 : 0) | (m_sq2.m_active ? 0x02 : 0) |
                   (m_wave.m_active ? 0x04 : 0) | (m_noise.m_active ? 0x08 : 0);
        } else {
            return 0x70;
        }
        break;
    }
    return 0xff;
}

template<>
u8 Audio::RegAccess<Access::Write>(const u8 addr, const u8 v)
{

    if (addr >= 0x30 && addr < 0x40) {
        AUD3WAVERAM[addr - 0x30] = v;
        return 0xff;
    }

    if (!m_master_enable && addr != 0x26) {
        return 0xff;
    }

    switch (addr) {
    case 0x10:
        NR10                = v;
        m_sq1.m_sweep.freq  = (v >> 4) & 0x7;
        m_sq1.m_sweep.inc   = (v & 8) == 0;
        m_sq1.m_sweep.shift = v & 7;
        break;
    case 0x11:
        NR11            = v;
        m_sq1.m_sq.duty = v >> 6;
        m_sq1.m_lenght  = 64 - (int)(v & 63);
        break;
    case 0x12:
        NR12                 = v;
        m_sq1.m_env.init_vol = v >> 4;
        m_sq1.m_env.inc      = (v & 0x08) != 0;
        m_sq1.m_env.freq     = v & 7;
        m_sq1.m_dac_active   = (v & 0xF8) != 0;
        if (!m_sq1.m_dac_active) {
            m_sq1.m_active = false;
        }
        break;
    case 0x13:
        NR13 = v; // TODO : coud be removed since write only

        m_sq1.m_freq = (m_sq1.m_freq & 0x700) | v;
        break;
    case 0x14:
        NR14 = v;

        m_sq1.m_freq       = (m_sq1.m_freq & 0xff) | ((v & 7) << 8);
        m_sq1.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80) {
            m_sq1.m_active     = m_sq1.m_env.init_vol || m_sq1.m_env.inc;
            m_sq1.m_period     = 2048 - m_sq1.m_freq;
            m_sq1.m_env.period = m_sq1.m_env.freq;
            m_sq1.m_env.vol    = m_sq1.m_env.init_vol;
            if (m_sq1.m_sweep.freq || m_sq1.m_sweep.shift) {
                m_sq1.m_sweep.period   = m_sq1.m_sweep.freq;
                m_sq1.m_sweep.freq_int = m_sq1.m_freq;
                m_sq1.m_sweep.active   = true;
                if (m_sq1.m_sweep.shift) {
                    const bool DONT_WRITE_FREQ_REG = false;
                    m_sq1.ComputeSweep(DONT_WRITE_FREQ_REG);
                }
            }
        }
        break;
    case 0x16:
        NR21 = v;

        m_sq2.m_sq.duty = v >> 6;
        m_sq2.m_lenght  = 64 - (int)(v & 63);
        break;
    case 0x17:
        NR22 = v;

        m_sq2.m_env.init_vol = v >> 4;
        m_sq2.m_env.inc      = (v & 0x08) != 0;
        m_sq2.m_env.freq     = v & 7;
        m_sq2.m_dac_active   = (v & 0xF8) != 0;
        if (!m_sq2.m_dac_active) {
            m_sq2.m_active = false;
        }
        break;
    case 0x18:
        NR23 = v; // TODO : coud be removed since write only

        m_sq2.m_freq = (m_sq2.m_freq & 0x700) | v;
        break;
    case 0x19:
        NR24 = v;

        m_sq2.m_freq       = (m_sq2.m_freq & 0xff) | ((v & 7) << 8);
        m_sq2.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80) {
            m_sq2.m_active     = m_sq2.m_env.init_vol || m_sq2.m_env.inc;
            m_sq2.m_period     = 2048 - m_sq2.m_freq;
            m_sq2.m_env.period = m_sq2.m_env.freq;
            m_sq2.m_env.vol    = m_sq2.m_env.init_vol;
        }
        break;
    case 0x1a:
        NR30 = v;

        m_wave.m_dac_active = (v & 0x80) != 0;
        if (!m_wave.m_dac_active) {
            m_wave.m_active = false;
        }
        break;
    case 0x1b:
        NR31 = v; // TODO : could be removed

        m_wave.m_lenght = 256 - (int)v;
        break;
    case 0x1c:
        NR32 = v;

        switch ((v >> 5) & 3) {
        case 0: m_wave.m_vol_shift = 4; break;
        case 1: m_wave.m_vol_shift = 0; break;
        case 2: m_wave.m_vol_shift = 1; break;
        case 3: m_wave.m_vol_shift = 2; break;
        }
        m_wave.m_out_val = m_wave.m_cur_val >> m_wave.m_vol_shift;
        break;
    case 0x1d:
        NR33 = v; // TODO : coud be removed since write only

        m_wave.m_freq = (m_wave.m_freq & 0x700) | v;
        break;
    case 0x1e:
        NR34 = v;

        m_wave.m_freq       = (m_wave.m_freq & 0xff) | ((v & 7) << 8);
        m_wave.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80) {
            m_wave.m_active = m_wave.m_dac_active;
            m_wave.m_period = 2048 - m_wave.m_freq;
            m_wave.m_phase  = 0;
        }
        break;
    case 0x20:
        NR41 = v;

        m_noise.m_lenght = 64 - (int)(v & 63);
        break;
    case 0x21:
        NR42 = v;

        m_noise.m_env.init_vol = v >> 4;
        m_noise.m_env.inc      = (v & 0x08) != 0;
        m_noise.m_env.freq     = v & 7;
        m_noise.m_dac_active   = (v & 0xF8) != 0;
        if (!m_noise.m_dac_active) {
            m_noise.m_active = false;
        }
        break;
    case 0x22:
        NR43 = v;

        m_noise.m_freq     = (v & 0xF0) >> 4;
        m_noise.m_use7bits = (v & 8) != 0;
        m_noise.m_div      = (v & 7) << 2;
        if (m_noise.m_div == 0) {
            m_noise.m_div = 2;
        }
        m_noise.m_period = (int)m_noise.m_div << (int)m_noise.m_freq;
        break;
    case 0x23:
        NR44 = v;

        m_noise.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80) {
            m_noise.m_active     = m_noise.m_env.init_vol || m_noise.m_env.inc;
            m_noise.m_env.period = m_noise.m_env.freq;
            m_noise.m_env.vol    = m_noise.m_env.init_vol;
            m_noise.m_lfsr       = 0xFFFF;
        }
        break;
    case 0x24:
        NR50 = v;

        m_right_vol = ((v & 0x70) >> 4) + 1;
        m_left_vol  = (v & 7) + 1;
        break;
    case 0x25: NR51 = v; break;
    case 0x26:
        NR52 = v;

        m_master_enable = (v & 0x80) != 0;
        if (!m_master_enable) {
            m_sequencer_clock = 0;
            m_sq1.Reset(*this);
            m_sq2.Reset(*this);
            m_wave.Reset(*this);
            m_noise.Reset(*this);
        }
        break;
    }

    return 0xff;
}

void SoundChannel::Reset(Audio& rAudio)
{
    m_active     = false;
    m_dac_active = false;
    m_freq       = 0;
    m_lenght     = 0;
    m_use_lenght = true;
    m_out_val    = 0;
    m_cur_wave   = 0;
    m_period     = 0;

    rAudio.NR50 = 0;
    rAudio.NR51 = 0;
}

void SoundChannel::RunLenght()
{
    if (m_active && m_use_lenght) {
        if (m_lenght > 0) {
            --m_lenght;
            if (m_lenght == 0) {
                m_active = false;
            }
        }
    }
}

void SoundChannel::RunEnv(SoundEnv& rEnv)
{
    if (!m_active) {
        return;
    }

    if (rEnv.period > 0 && rEnv.freq) {
        --rEnv.period;
        if (rEnv.period == 0) {
            if (rEnv.inc) {
                if (rEnv.vol < 15) {
                    ++rEnv.vol;
                }

                if (rEnv.vol < 15) {
                    rEnv.period = rEnv.freq;
                } else {
                    rEnv.period = 0;
                }
            } else {
                if (rEnv.vol > 0) {
                    --rEnv.vol;
                }

                if (rEnv.vol > 0) {
                    rEnv.period = rEnv.freq;
                } else {
                    rEnv.period = 0;
                    m_active    = false;
                }
            }
        }
    }
}

void SoundChannel::RunSquare(SquareWave& rSq)
{
    if (!m_active) {
        return;
    }

    if (m_period == 0) {
        bool out  = false;
        rSq.phase = (rSq.phase + 1) & 7;
        switch (rSq.duty) {
        case 0: out = rSq.phase == 7; break;
        case 1: out = rSq.phase == 0 || rSq.phase == 7; break;
        case 2: out = rSq.phase == 0 || rSq.phase >= 5; break;
        case 3: out = rSq.phase != 0 && rSq.phase != 7; break;
        }
        m_out_val = out ? 1 : 0;
        m_period  = 2048 - m_freq;
    } else {
        --m_period;
    }
}

void Square1::Reset(Audio& rAudio)
{
    SoundChannel::Reset(rAudio);

    m_sweep.active   = false;
    m_sweep.freq     = 0;
    m_sweep.freq_int = 0;
    m_sweep.inc      = false;
    m_sweep.period   = 0;
    m_sweep.shift    = 0;
    m_sq.duty        = 0;
    m_sq.phase       = 0;

    rAudio.NR10 = 0;
    rAudio.NR11 = 0;
    rAudio.NR12 = 0;
    rAudio.NR13 = 0;
    rAudio.NR14 = 0;
}

void Square1::RunSweep()
{
    if (!m_active || !m_sweep.active) {
        return;
    }

    if (m_sweep.period <= 1) {
        const bool WRITE_TO_FREQ_REG = true;
        ComputeSweep(WRITE_TO_FREQ_REG);

        m_sweep.period = m_sweep.freq;
    } else {
        --m_sweep.period;
    }
}

void Square1::ComputeSweep(bool write_freq_reg)
{
    const int adjust = (int)m_sweep.freq_int >> m_sweep.shift;
    if (m_sweep.inc) {
        int new_freq = (int)m_freq + adjust;
        if (new_freq >= 0x800) {
            m_sweep.active = false;
            m_active       = false;
        } else if (write_freq_reg && m_sweep.shift) {
            m_freq   = (u16)(new_freq & 0x7FF);
            m_period = 2048 - m_freq;
        }
    } else {
        int new_freq = (int)m_freq - adjust;
        if (new_freq < 0) {
            m_sweep.active = false;
            m_active       = false;
        } else if (write_freq_reg && m_sweep.shift) {
            m_freq   = (u16)(new_freq & 0x7FF);
            m_period = 2048 - m_freq;
        }
    }
}

void Square2::Reset(Audio& rAudio)
{
    SoundChannel::Reset(rAudio);

    m_sq.duty  = 0;
    m_sq.phase = 0;

    rAudio.NR21 = 0;
    rAudio.NR22 = 0;
    rAudio.NR23 = 0;
    rAudio.NR24 = 0;
}

void SoundWave::Reset(Audio& rAudio)
{
    SoundChannel::Reset(rAudio);

    m_vol_shift = 0;
    m_phase     = 0;
    m_cur_wave  = 0;
    m_cur_val   = 0;

    rAudio.NR30 = 0;
    rAudio.NR31 = 0;
    rAudio.NR32 = 0;
    rAudio.NR33 = 0;
    rAudio.NR34 = 0;
}

void SoundWave::RunWave(Audio& rAudio)
{
    if (!m_active) {
        return;
    }

    if (m_period == 0) {
        m_phase = (m_phase + 1) & 31;

        m_cur_wave = rAudio.AUD3WAVERAM[m_phase >> 1];
        m_cur_val  = m_cur_wave;
        if (m_phase & 1) {
            m_cur_val = m_cur_val & 0x0F;
        } else {
            m_cur_val = m_cur_val >> 4;
        }

        m_out_val = m_cur_val >> m_vol_shift;

        m_period = 2048 - m_freq;
    } else {
        --m_period;
    }
}

void Noise::Reset(Audio& rAudio)
{
    SoundChannel::Reset(rAudio);

    m_use7bits = false;
    m_div      = 0;
    m_lfsr     = 0xff;

    rAudio.NR41 = 0;
    rAudio.NR42 = 0;
    rAudio.NR43 = 0;
    rAudio.NR44 = 0;
}

void Noise::RunNoise()
{
    if (!m_active) {
        return;
    }

    if (m_period == 0) {
        m_period = (int)m_div << (int)m_freq;
        if (m_freq < 14) {
            if (m_use7bits) {
                m_out_val = (((m_lfsr & 0x40) >> 6) ^ 1);
                m_lfsr    = (m_lfsr << 1) | (((m_lfsr & 0x40) >> 6) ^ ((m_lfsr & 0x20) >> 5));
            } else {
                m_out_val = (((m_lfsr & 0x4000) >> 14) ^ 1);
                m_lfsr    = (m_lfsr << 1) | (((m_lfsr & 0x4000) >> 14) ^ ((m_lfsr & 0x2000) >> 13));
            }
        }
    } else {
        --m_period;
    }
}

} // namespace GB
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

#include "GB.h"
#include "GB_Types.h"

namespace GB {

struct SoundEnv
{
    bool inc = false;
    u8 period = 0;
    u8 freq = 0;
    u8 init_vol = 0;
    u8 vol = 0;
};

struct SquareWave
{
    u8 phase = 0;
    u8 duty = 0;
};

struct FreqSweep
{
    bool active = false;
    bool inc = false;
    int freq_int = 0;
    int period = 0;
    u8 freq = 0;
    u8 shift = 0;
};

class SoundChannel
{
public:
    bool m_active = false;
    bool m_dac_active = false;
    int m_lenght = 0;
    bool m_use_lenght = false;
    u8 m_out_val = 0;
    u8 m_cur_wave = 0;
    u8 m_cur_val = 0;
    u16 m_freq = 0;
    int m_period = 0;

    void Reset(Audio& rAudio);
    void RunLenght();
    void RunEnv(SoundEnv& rEnv);
    void RunSquare(SquareWave& rSq);
};

class Square1 : public SoundChannel
{
public:
    SoundEnv m_env;
    SquareWave m_sq;
    FreqSweep m_sweep;

    void Reset(Audio& rAudio);
    void RunSweep();
    void ComputeSweep(bool write_freq_reg);
};

class Square2 : public SoundChannel
{
public:
    SoundEnv m_env;
    SquareWave m_sq;

    void Reset(Audio& rAudio);
};

class SoundWave : public SoundChannel
{
public:
    u8 m_phase = 0;
    u8 m_vol_shift = 0;

    void Reset(Audio& rAudio);
    void RunWave(Audio& rAudio);
};

class Noise : public SoundChannel
{
public:
    SoundEnv m_env;

    bool m_use7bits = 0;
    u16  m_lfsr = 0;
    u8   m_div = 0;

    void Reset(Audio& rAudio);
    void RunNoise();
};

class Audio
{
public:

    u8 NR10 = 0;
    u8 NR11 = 0;
    u8 NR12 = 0;
    u8 NR13 = 0;
    u8 NR14 = 0;
    u8 NR21 = 0;
    u8 NR22 = 0;
    u8 NR23 = 0;
    u8 NR24 = 0;
    u8 NR30 = 0;
    u8 NR31 = 0;
    u8 NR32 = 0;
    u8 NR33 = 0;
    u8 NR34 = 0;
    u8 NR41 = 0;
    u8 NR42 = 0;
    u8 NR43 = 0;
    u8 NR44 = 0;
    u8 NR50 = 0;
    u8 NR51 = 0;
    u8 NR52 = 0;
    std::array<u8, 16> AUD3WAVERAM = {};

    static constexpr size_t AUDIO_BUF_SIZE_POW2 = 13;
    static constexpr size_t AUDIO_BUF_SIZE = (1 << AUDIO_BUF_SIZE_POW2);
    static constexpr size_t AUDIO_BUF_SIZE_MASK = AUDIO_BUF_SIZE - 1;

    std::array<AudioSample, AUDIO_BUF_SIZE> m_audio_buf = {};

    size_t m_audio_pos = 0;
    size_t m_audio_clock = 0;
    u8 m_sequencer_clock = 0;

    Square1 m_sq1;
    Square2 m_sq2;
    SoundWave m_wave;
    Noise m_noise;

    bool m_master_enable = false;

    i16 m_left = 0;
    i16 m_right = 0;

    u8 m_left_vol = 0;
    u8 m_right_vol = 0;

    i32 m_left_cap = 0;
    i32 m_right_cap = 0;

    void Reset(ResetOption reset_opt);
    void Tick(System&);

    template<Access eAccess> u8 RegAccess(const u8 addr, const u8 v = 0);
};

template<>
u8 Audio::RegAccess<Access::Read>(const u8 addr, const u8 v);

template<>
u8 Audio::RegAccess<Access::Write>(const u8 addr, const u8 v);

} // namespace GB

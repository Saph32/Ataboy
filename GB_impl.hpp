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

GB::GB()
    : m_pSystem(new System)
{
}

GB::~GB()
{
}

bool GB::Load(const char * pRom_data, size_t size)
{
    return m_pSystem->m_game_pak.Load(pRom_data, size);
}

bool GB::LoadSaveRAM(const char * pRom_data, size_t size)
{
    return m_pSystem->m_game_pak.LoadSaveRAM(pRom_data, size);
}

bool GB::LoadBootROM(const char * pROM_data, size_t size)
{
    if (size != 256) {
        return false;
    }

    memcpy(m_pSystem->m_boot_ROM.data(), pROM_data, m_pSystem->m_boot_ROM.size());

    return true;
}

void GB::Reset(ResetOption reset_opt)
{
    m_pSystem->Reset(reset_opt);
}

void GB::RunFrame()
{
    m_pSystem->RunFrame();
}

std::chrono::nanoseconds GB::RunTime(const std::chrono::nanoseconds& time_to_run)
{
    return m_pSystem->RunTime(time_to_run);
}

u32 GB::GetFrameNumber()
{
    return m_pSystem->m_frame_number;
}

const u32 * GB::GetPixels() const
{
    return m_pSystem->m_video.m_upBack_buf->pix.data();
}

void GB::UpdateKeys(const Keys & rKeys)
{
    m_pSystem->m_io.m_keys = rKeys;
    m_pSystem->m_io.MakeP1(*m_pSystem);
}

const AudioSample * GB::GetAudioBuf() const
{
    return m_pSystem->m_audio.m_audio_buf.data();
}

const size_t GB::GetAudioBufSize() const
{
    return m_pSystem->m_audio.m_audio_buf.size();
}

size_t GB::GetAudioBufPos() const
{
    return m_pSystem->m_audio.m_audio_pos;
}

const ROM_Header & GB::RefHeader() const
{
    return m_pSystem->m_game_pak.m_public_header;
}

std::pair<const char*, size_t> GB::RefSaveRAM() const
{
    return {(const char*)(m_pSystem->m_game_pak.RAM.data()), m_pSystem->m_game_pak.m_RAM_size};
}

}
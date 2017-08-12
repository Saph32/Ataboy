// Copyright (c) 2016, Jean Charles Vallieres
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

#include <memory>
#include <chrono>
#include <string>
#include <utility>

namespace GB
{

typedef unsigned char   u8;
typedef unsigned short  u16;
typedef unsigned int    u32;
typedef unsigned long long int u64;
typedef signed short i16;
typedef signed int i32;

struct ROM_Header
{
    std::string title;
    size_t RAM_size;
};

class System;

union Keys {
    struct k {
        u8 right : 1;
        u8 left : 1;
        u8 up : 1;
        u8 down : 1;
        u8 a : 1;
        u8 b : 1;
        u8 select : 1;
        u8 start : 1;
    } k; u8 value = 0;
};

struct AudioSample
{
    i16 left;
    i16 right;
};

enum ResetOption
{
    ResetOption_NONE            = 0x00,
    ResetOption_USE_BOOT_ROM    = 0x01,
};

class GB
{
public:
    
    GB();
    ~GB();

    bool Load(const char* pROM_data, size_t size);
    bool LoadSaveRAM(const char* pRAM_data, size_t size);
    bool LoadBootROM(const char* pROM_data, size_t size);

    void Reset(ResetOption reset_opt);

    void RunFrame();
    std::chrono::nanoseconds RunTime(const std::chrono::nanoseconds& time_to_run);
    u32 GetFrameNumber();
    const u32* GetPixels() const;
    void UpdateKeys(const Keys& rKeys);
    const AudioSample* GetAudioBuf() const;
    const size_t GetAudioBufSize() const;
    size_t GetAudioBufPos() const;

    const ROM_Header& RefHeader() const;
    std::pair<const char*, size_t> RefSaveRAM() const;

private:

    std::unique_ptr<System> m_pSystem;
};

} // namespace GB
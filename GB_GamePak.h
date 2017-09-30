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

#include <vector>
#include <array>

#include "GB.h"
#include "GB_Types.h"

namespace GB {
class GamePak {
public:
    enum class MBC {
        None = 0,
        MBC1 = 1,
        MBC2 = 2,
        MBC3 = 3,
        MBC5 = 5,
    };

    std::vector<u8> ROM;
    std::vector<u8> RAM;

    union {
        struct {
            u8  EntryPoint[4];
            u8  Logo[48];
            u8  Title[16];
            u8  NewLicenseCode[2];
            u8  SGBFlag;
            u8  CartType;
            u8  ROMSize;
            u8  RAMSize;
            u8  DestinationCode;
            u8  OldLicenseCode;
            u8  MaskROMVersion;
            u8  HeaderChecksum;
            u16 GlobalChecksum;
        } fields;
        std::array<u8, 80> raw = {};
    } m_header = {};

    size_t m_ROM_size       = 0;
    size_t m_RAM_size       = 0;
    size_t m_ROM_bank_count = 0;
    size_t m_RAM_bank_count = 0;

    MBC m_MBC = MBC::None;

    u8 m_ROM_bank_select           = 0;
    u8 m_RAM_bank_select           = 0;
    u8 m_MBC1_extra_select         = 0;
    u8 m_MBC5_high_ROM_bank_select = 0;

    enum class MBC1ExtraMode {
        ROM = 0,
        RAM = 1,
    };

    MBC1ExtraMode m_MBC1_extra_mode = MBC1ExtraMode::ROM;

    size_t m_cur_ROM_bank = 0;
    size_t m_cur_RAM_bank = 0;

    size_t m_ROM_bank_mask = 0;
    size_t m_RAM_bank_mask = 0;

    const u8* m_pCur_ROM_bank = nullptr;
    const u8* m_pCur_RAM_bank = nullptr;

    bool m_ram_enabled = false;

    ROM_Header m_public_header = {};

    template<Access eAccess>
    u8 ROMAccess(const u16 addr, const u8 v = 0);
    template<Access eAccess>
    u8 RAMAccess(const u16 addr, const u8 v = 0);

    bool Load(const char* pROM_data, size_t size);
    bool LoadSaveRAM(const char* pRAM_data, size_t size);
};
} // namespace GB
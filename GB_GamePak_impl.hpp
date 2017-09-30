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

bool GamePak::Load(const char* pROM_data, size_t size)
{
    auto fnRead = [&](auto dest, size_t src_offset, size_t read_size) -> bool {
        if (src_offset + read_size <= size) {
            memcpy(dest, pROM_data + src_offset, read_size);
            return true;
        } else {
            return false;
        }
    };

    if (!fnRead(&m_header, 0x100, m_header.raw.size())) {
        printf("ERROR : Invalid header\n");
        return false;
    }

    char title[sizeof(m_header.fields.Title) + 1] = {0};

    memcpy(title, m_header.fields.Title, sizeof(m_header.fields.Title));

    printf("Title : %s\n", title);

    switch (m_header.fields.CartType) {
    case 0x00: m_MBC = MBC::None; break;
    case 0x01:
    case 0x02:
    case 0x03: m_MBC = MBC::MBC1; break;
    case 0x05:
    case 0x06: m_MBC = MBC::MBC2; break;
    case 0x0f:
    case 0x10:
    case 0x11:
    case 0x12:
    case 0x13: m_MBC = MBC::MBC3; break;
    case 0x19:
    case 0x1a:
    case 0x1b:
    case 0x1c:
    case 0x1d:
    case 0x1e: m_MBC = MBC::MBC5; break;
    default: printf("Error : Invalid MBC\n"); return false;
    }

    printf("MBC : %d\n", static_cast<int>(m_MBC));

    if (m_header.fields.ROMSize >= 9) {
        printf("Error : Invalid ROM size\n");
        return false;
    }

    switch (m_header.fields.RAMSize) {
    case 0: m_RAM_size = 0; break;
    case 1: m_RAM_size = 2 * 1024; break;
    case 2: m_RAM_size = 8 * 1024; break;
    case 3: m_RAM_size = 32 * 1024; break;
    default: return false;
    }

    if (m_MBC == MBC::MBC2) {
        m_RAM_size = 512;
    }

    m_RAM_bank_count = 0;
    if (m_RAM_size > 0) {
        // TODO : test behavior of RAM < 8K. Does it mirror ?
        const size_t ram_size = max(m_RAM_size, size_t(8 * 1024));
        RAM.resize(ram_size);
        m_RAM_bank_count = ram_size / (8 * 1024);
        m_RAM_bank_mask  = m_RAM_bank_count - 1;
        // TODO : fill ram with garbage
    } else {
        // Allocate some dummy ram space
        RAM.resize(8 * 1024);
        m_RAM_bank_mask = 0;
    }

    const size_t rom_size = (32 * 1024ULL) << m_header.fields.ROMSize;
    ROM.resize(rom_size);

    if (!fnRead(ROM.data(), 0, rom_size)) {
        printf("Error : ROM too small\n");
        return false;
    }

    m_pCur_ROM_bank  = &ROM[0x4000];
    m_ROM_bank_count = rom_size / (16 * 1024);
    m_ROM_bank_mask  = m_ROM_bank_count - 1;

    m_public_header.title    = title;
    m_public_header.RAM_size = m_RAM_size;

    return true;
}

bool GamePak::LoadSaveRAM(const char* pRAM_data, size_t size)
{
    if (m_RAM_size != size) {
        printf("ERROR : Incorrect SaveRAM size\n");
        return false;
    }

    memcpy(RAM.data(), pRAM_data, min(size, RAM.size()));
    return true;
}

template<>
u8 GamePak::ROMAccess<Access::Write>(const u16 addr, const u8 v)
{

    auto fnUpdateCurROMBank = [this]() {
        m_cur_ROM_bank &= m_ROM_bank_mask;
        m_pCur_ROM_bank = &ROM[m_cur_ROM_bank << 14];
    };

    auto fnUpdateCurRAMBank = [this]() {
        m_cur_RAM_bank &= m_RAM_bank_mask;
        m_pCur_RAM_bank = &RAM[m_cur_RAM_bank << 13];
    };

    auto fnComputeMBC1ROMBank = [this]() {
        m_cur_ROM_bank = m_ROM_bank_select;
        if (m_MBC1_extra_mode == MBC1ExtraMode::ROM) {
            m_cur_ROM_bank += ((size_t)m_MBC1_extra_select << 5);
        }
    };

    if (addr <= 0x1FFF) {
        m_ram_enabled = ((v & 0xF) == 0x0A);
    } else if (addr <= 0x3fff) {
        switch (m_MBC) {
        case MBC::None:
        default: break;
        case MBC::MBC1:
            m_ROM_bank_select = v & 0x1F;
            if (m_ROM_bank_select == 0) {
                m_ROM_bank_select = 1;
            }
            fnComputeMBC1ROMBank();
            fnUpdateCurROMBank();
            break;
        case MBC::MBC2:
            m_ROM_bank_select = v & 0xF;
            m_cur_ROM_bank    = m_ROM_bank_select;
            fnUpdateCurROMBank();
            break;
        case MBC::MBC3:
            m_ROM_bank_select = v & 0x7F;
            if (m_ROM_bank_select == 0) {
                m_ROM_bank_select = 1;
            }
            m_cur_ROM_bank = m_ROM_bank_select;
            fnUpdateCurROMBank();
            break;
        case MBC::MBC5:
            if (addr <= 0x2fff) {
                m_ROM_bank_select = v;
            } else {
                m_MBC5_high_ROM_bank_select = v & 1;
            }
            m_cur_ROM_bank = m_ROM_bank_select;
            m_cur_ROM_bank += (size_t)m_MBC5_high_ROM_bank_select << 8;
            fnUpdateCurROMBank();
            break;
        }
    } else if (addr <= 0x5fff) {
        switch (m_MBC) {
        case MBC::None:
        case MBC::MBC2:
        default: break;
        case MBC::MBC1:
            m_MBC1_extra_select = v & 0x03;
            if (m_MBC1_extra_mode == MBC1ExtraMode::ROM) {
                fnComputeMBC1ROMBank();
                fnUpdateCurROMBank();
            } else {
                m_cur_RAM_bank = m_MBC1_extra_select;
                fnUpdateCurRAMBank();
            }
            break;
        case MBC::MBC3:
            if (v < 4) {
                m_RAM_bank_select = v;
                m_cur_RAM_bank    = m_RAM_bank_select;
                fnUpdateCurRAMBank();
            }
            break;
        case MBC::MBC5:
            m_RAM_bank_select = v & 0x03;
            m_cur_RAM_bank    = m_RAM_bank_select;
            fnUpdateCurRAMBank();
            break;
        }
    } else if (addr <= 0x7fff) {
        if (m_MBC == MBC::MBC1) {
            m_MBC1_extra_mode = (v & 1) ? MBC1ExtraMode::RAM : MBC1ExtraMode::ROM;
            fnComputeMBC1ROMBank();
            fnUpdateCurROMBank();
            m_cur_RAM_bank = (v & 1) ? m_MBC1_extra_select : 0;
            fnUpdateCurRAMBank();
        }
    }

    return 0;
}

template<>
u8 GamePak::ROMAccess<Access::Read>(const u16 addr, const u8)
{
    if (addr < 0x4000) {
        return ROM[addr];
    } else {
        return m_pCur_ROM_bank[addr & 0x3fff];
    }
}

template<Access eAccess>
u8 GamePak::RAMAccess(const u16 addr, const u8 v)
{
#pragma warning(push)
#pragma warning(disable : 4127) // warning C4127: conditional expression is constant

    if (m_ram_enabled) {
        if (eAccess == Access::Write) {
            RAM[addr & 0x1fff] = v;
        } else {
            return RAM[addr & 0x1fff];
        }
    }

    return 0xFF;
#pragma warning(pop)
}

} // namespace GB

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

#include "GB.h"

#include <array>
#include <memory>
#include <algorithm>
#include <vector>

#include <cstdio>
#include <cstring>

using namespace std;
using namespace std::chrono;

namespace GB
{

enum class Access
{
    Read,
    Write,
};

class System;
class CPU;
class GamePak;
class IO;
class Video;
class Audio;

class CPU
{
public:

    enum Operand8 { OpA, OpB, OpC, OpD, OpE, OpH, OpL, OpIHL, OpImm, OpIBC, OpIDE, OpIoImm, OpIoC, OpHLI, OpHLD, OpIImm };
    enum Operand16 { OpBC, OpDE, OpHL, OpSP, OpAF, OpImm16 };
    enum OpALU { ADD, ADC, SUB, SBC, CP };
    enum OpBITW { AND, OR, XOR };
    enum OpBITS { RL, RLA, RLC, RLCA, RR, RRA, RRC, RRCA, SLA, SRA, SRL, SWP };
    enum CondFlag { Always, CondZ, CondNZ, CondC, CondNC, RETI };

    typedef void (CPU::*OpCodeFn)(System&);

    // Registers
    struct CPURegs
    {
        union { struct { u8 C; u8 B; } bc8; u16 BC; };
        union { struct { u8 E; u8 D; } de8; u16 DE; };
        union { struct { u8 L; u8 H; } hl8; u16 HL; };
        union { struct { u8 FLAGS; u8 A; } af8; u16 AF; };
        struct Flags {    // Optimization : use a full byte for each flag
            u8 C;    // Carry
            u8 H;    // Half-Carry
            u8 N;    // Substract
            u8 Z;    // Zero
        } F;
        u16 PC;
        u16 SP;
    } R = {};

    array<u8, 128> HRAM = {};

    // Internal state
    bool IME = false;    // Interrupt enable

    union IFLAG {
        struct Flags {
            u8 vblank : 1;
            u8 lcdstat : 1;
            u8 timer : 1;
            u8 serial : 1;
            u8 joypad : 1;
        } f;
        u8 value = 0;
    };

    IFLAG& IE = (IFLAG&)HRAM[127];
    IFLAG IF = {};

    bool m_halt = false;
    bool m_ei_delay = false;
    bool m_oam_dma_active = false;
    u16 m_oam_dma_src = 0;
    u8 m_oam_dma_index = 0;

    static const array<OpCodeFn, 256> m_op_codes;
    static const array<OpCodeFn, 256> m_op_codes_CB;

    void Reset();

    u8   RB(System& rSystem, const u16 addr);
    void WB(System& rSystem, const u16 addr, const u8 v);

    template<Operand8 op8> u8 GetOperand8(System& rSystem);
    template<Operand8 op8> void SetOperand8(System& rSystem,const u8 v);
    template<Operand16 op16> u16 GetOperand16(System& rSystem);
    template<Operand16 op16> void SetOperand16(const u16 v);

    void NOP(System& rSystem);
    template<Operand8 src> void INC(System& rSystem);
    template<Operand16 src> void INC16(System& rSystem);
    template<Operand16 src> void DEC16(System& rSystem);
    template<Operand8 src> void DEC(System& rSystem);
    template<Operand8 src, OpALU alu> void ALU(System& rSystem);
    template<Operand16 src> void ADDHL(System& rSystem);
    template<Operand8 src, OpBITW op> void BITW(System& rSystem);
    template<Operand8 src, OpBITS op> void BITS(System& rSystem);
    template<Operand8 src, u8 bit> void SET(System& rSystem);
    template<Operand8 src, u8 bit> void RES(System& rSystem);
    template<Operand8 src, u8 bit> void BIT(System& rSystem);
    template<Operand8 src, Operand8 dst> void LD(System& rSystem);
    template<Operand16 src, Operand16 dst> void LD16(System& rSystem);
    void LDSP(System& rSystem);
    template<Operand16 dst> void LDSPOF(System& rSystem);
    void HALT(System& rSystem);
    void STOP(System& rSystem);
    void CB(System& rSystem);
    void SCF(System& rSystem);
    void CCF(System& rSystem);
    template<Operand16 src> void PUSH(System& rSystem);
    template<Operand16 dst> void POP(System& rSystem);
    template<CondFlag con> void CALL(System& rSystem);
    template<u8 addr> void RST(System& rSystem);
    template<CondFlag con> void RET(System& rSystem);
    template<CondFlag con> void JP(System& rSystem);
    template<CondFlag con> void JR(System& rSystem);
    void JPHL(System& rSystem);
    void EI(System& rSystem);
    void DI(System& rSystem);
    void CPL(System& rSystem);
    void DAA(System& rSystem);

    template<CondFlag con> bool Condition() const;
    void PUSH16(System& rSystem, const u16 v);
    u16 POP16(System& rSystem);

    void DoOAMDMA(System& rSystem);

    void Execute(System& rSystem);
};

class Video
{
public:

    struct FrameBuf {
        array<u32, 160 * 144> pix = {};
    };

    array<u8, 8 * 1024> VRAM = {};
    array<u8, 256> OAM = {};

    union {
        struct {
            u8 bgdisplay : 1;
            u8 objdisplay : 1;
            u8 objsize : 1;
            u8 bgtilemap : 1;
            u8 bgwintiledata : 1;
            u8 windisp : 1;
            u8 wintilemap : 1;
            u8 lcden : 1;
        } bits; u8 value = 0;
    } LCDC = {};

    union {
        struct {
            u8 mode : 2;
            u8 coincidence : 1;
            u8 mode0int : 1;
            u8 mode1int : 1;
            u8 mode2int : 1;
            u8 coincidenceint : 1;
        } bits; u8 value = 0;
    } STAT = {};

    u8 SCY = 0;
    u8 SCX = 0;
    u8 LY = 0;
    u8 LYC = 0;
    u8 BGP = 0;
    u8 OBP0 = 0;
    u8 OBP1 = 0;
    u8 WY = 0;
    u8 WX = 0;

    u8 LCDX = 0;

    u8 m_mode = 0;

    unique_ptr<FrameBuf> m_upFront_buf;
    unique_ptr<FrameBuf> m_upBack_buf;

    void Reset();

    void Tick(System& rSystem);

    void Flip(System& rSystem);
    void RenderLine();

    template<Access eAccess> u8 VRAMAccess(const u16 addr, const u8 v = 0);
    template<Access eAccess> u8 OAMAccess(const u8 addr, const u8 v = 0);

    void UpdateSTAT() { 
        STAT.bits.coincidence = (LY == LYC) ? 1 : 0;
        STAT.bits.mode = m_mode;
    }
};

class GamePak
{
public:

    enum class MBC
    {
        None = 0,
        MBC1 = 1,
        MBC2 = 2,
        MBC3 = 3,
        MBC5 = 5,
    };

    vector<u8> ROM;
    vector<u8> RAM;

    union
    {
        struct
        {
            u8 EntryPoint[4];   u8 Logo[48];        u8 Title[16];       u8 NewLicenseCode[2];
            u8 SGBFlag;         u8 CartType;        u8 ROMSize;         u8 RAMSize;
            u8 DestinationCode; u8 OldLicenseCode;  u8 MaskROMVersion;  u8 HeaderChecksum;
            u16 GlobalChecksum;
        } fields;
        array<u8, 80> raw = {};
    } m_header = {};

    size_t m_ROM_size = 0;
    size_t m_RAM_size = 0;
    size_t m_ROM_bank_count = 0;
    size_t m_RAM_bank_count = 0;

    MBC m_MBC = MBC::None;

    u8 m_ROM_bank_select = 0;
    u8 m_RAM_bank_select = 0;
    u8 m_MBC1_extra_select = 0;
    u8 m_MBC5_high_ROM_bank_select = 0;

    enum class MBC1ExtraMode
    {
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

    template<Access eAccess> u8 ROMAccess(const u16 addr, const u8 v = 0);
    template<Access eAccess> u8 RAMAccess(const u16 addr, const u8 v = 0);

    bool Load(const char* pRom_data, size_t size);
};

class IO
{
public:

    u8 P1 = 0;
    u8 SB = 0;
    u8 SC = 0;
    union {
        struct DividerReg {
            u8 low;
            u8 DIV;
        } details;
        u16 value = 0;
    } Divider = {};

    u8 TIMA = 0;
    u8 TMA = 0;
    u8 TAC = 0;

    u8 m_timer_mask = 255;

    u8 m_serial_bits = 0;

    Keys m_keys = {};
    
    void Reset();

    void Tick(System& m_rSystem);

    u8 MakeP1(System& rSystem);
    void SetTAC(u8 v);

    template<Access eAccess> u8 RegAccess(System& rSystem, const u8 addr, const u8 v = 0);
};

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
    array<u8, 16> AUD3WAVERAM;

    static constexpr size_t AUDIO_BUF_SIZE_POW2 = 13;
    static constexpr size_t AUDIO_BUF_SIZE = (1 << AUDIO_BUF_SIZE_POW2);
    static constexpr size_t AUDIO_BUF_SIZE_MASK = AUDIO_BUF_SIZE - 1;

    array<AudioSample, AUDIO_BUF_SIZE> m_audio_buf = {};

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

    void Reset();
    void Tick(System&);

    template<Access eAccess> u8 RegAccess(const u8 addr, const u8 v = 0);
};

template<>
u8 Audio::RegAccess<Access::Read>(const u8 addr, const u8 v);

template<>
u8 Audio::RegAccess<Access::Write>(const u8 addr, const u8 v);

class System
{
public:

    CPU m_cpu;
    
    bool m_new_frame = false;
    u32 m_frame_number = 0;
    u64 m_cycle_count = 0;

    Video m_video;
    GamePak m_game_pak;
    IO m_io;
    Audio m_audio;

    array<u8, 8 * 1024> RAM = {};

    void Tick()
    {
        m_video.Tick(*this);
        m_io.Tick(*this);
        m_audio.Tick(*this);
        m_cpu.DoOAMDMA(*this);
        ++m_cycle_count;
    }

    void Reset();
    void RunFrame();
    nanoseconds RunTime(nanoseconds time_to_run);

    template<Access eAccess> u8 BusAccess(u16 addr, u8 v = 0);
};

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

void GB::Reset()
{
    m_pSystem->Reset();
}

void GB::RunFrame()
{
    m_pSystem->RunFrame();
}

std::chrono::nanoseconds GB::RunTime(std::chrono::nanoseconds time_to_run)
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

void CPU::Reset()
{
    R.PC = 0x100;
    R.SP = 0xFFFE;
    R.af8.A = 0x01;
    R.af8.FLAGS = 0xB0;
    R.F.Z = 1; R.F.N = 0; R.F.C = 1; R.F.H = 1;
    R.BC = 0x0013;
    R.DE = 0x00d8;
    R.HL = 0x014d;

    IF.value = 0;
    
    fill(begin(HRAM), end(HRAM), 0);

    //bQuit = false;
    m_halt = false;
    m_ei_delay = false;
    m_oam_dma_active = false;
    m_oam_dma_src = 0;
    m_oam_dma_index = 0;
}

inline u8 CPU::RB(System & rSystem, const u16 addr) {
    rSystem.Tick();
    return rSystem.BusAccess<Access::Read>(addr);
}

inline void CPU::WB(System & rSystem, const u16 addr, const u8 v) {
    rSystem.Tick();
    rSystem.BusAccess<Access::Write>(addr, v);
}


template<Access eAccess>
u8 System::BusAccess(u16 addr, u8 v) {
#pragma warning( push )
#pragma warning( disable : 4127 ) // warning C4127: conditional expression is constant
    if (addr < 0x8000) {
        return m_game_pak.ROMAccess<eAccess>(addr, v);
    }
    else if (addr < 0xa000) {
        return m_video.VRAMAccess<eAccess>(addr, v);
    }
    else if (addr < 0xc000) {
        return m_game_pak.RAMAccess<eAccess>(addr, v);
    }
    else if (addr < 0xfe00) {
        if (eAccess == Access::Write) {
            RAM[addr & 0x1fff] = v;
        }
        else {
            return RAM[addr & 0x1fff];
        }
    }
    else if (addr < 0xff00) {
        return m_video.OAMAccess<eAccess>(addr & 0xff, v);
    }
    else if (addr < 0xff80) {
        return m_io.RegAccess<eAccess>(*this, addr & 0xff, v);
    }
    else {
        if (eAccess == Access::Write) {
            m_cpu.HRAM[addr & 0x7f] = v;
        }
        else {
            return m_cpu.HRAM[addr & 0x7f];
        }
    }

    return 0;
#pragma warning( pop )
}

template<CPU::Operand8 op8>
u8 CPU::GetOperand8(System& rSystem)
{
    u8  t = 0;
    u16 t16 = 0;
    switch (op8) {
    case OpA: return R.af8.A;
    case OpB: return R.bc8.B;
    case OpC: return R.bc8.C;
    case OpD: return R.de8.D;
    case OpE: return R.de8.E;
    case OpH: return R.hl8.H;
    case OpL: return R.hl8.L;
    case OpIHL: return RB(rSystem, R.HL);
    case OpIBC: return RB(rSystem, R.BC);
    case OpIDE: return RB(rSystem, R.DE);
    case OpImm: R.PC++; return RB(rSystem, R.PC);
    case OpIoImm: R.PC++; t = RB(rSystem, R.PC); return RB(rSystem, 0xff00 | t);
    case OpIoC: return RB(rSystem, 0xff00 | R.bc8.C);
    case OpHLI: t = RB(rSystem, R.HL); R.HL++; return t;
    case OpHLD: t = RB(rSystem, R.HL); R.HL--; return t;
    case OpIImm: t16 = GetOperand16<OpImm16>(rSystem); return RB(rSystem, t16);
    }
    return 0;
}

template<CPU::Operand8 op8>
void CPU::SetOperand8(System& rSystem, const u8 v)
{
    u8 t = 0; 
    u16 t16 = 0;
    switch (op8) {
    case OpA: R.af8.A = v; break;
    case OpB: R.bc8.B = v; break;
    case OpC: R.bc8.C = v; break;
    case OpD: R.de8.D = v; break;
    case OpE: R.de8.E = v; break;
    case OpH: R.hl8.H = v; break;
    case OpL: R.hl8.L = v; break;
    case OpIHL: WB(rSystem, R.HL, v); break;
    case OpIBC: WB(rSystem, R.BC, v); break;
    case OpIDE: WB(rSystem, R.DE, v); break;
    case OpIoImm: R.PC++; t = RB(rSystem, R.PC); WB(rSystem, 0xff00 | t, v); break;
    case OpIoC: WB(rSystem, 0xff00 | R.bc8.C, v); break;
    case OpHLI: WB(rSystem, R.HL, v); R.HL++; break;
    case OpHLD: WB(rSystem, R.HL, v); R.HL--; break;
    case OpIImm: t16 = GetOperand16<OpImm16>(rSystem); WB(rSystem, t16, v); break;
    }
}

template<CPU::Operand16 op16>
u16 CPU::GetOperand16(System& rSystem)
{
    u16 t;
    switch (op16) {
    case OpBC: return R.BC;
    case OpDE: return R.DE;
    case OpHL: return R.HL;
    case OpSP: return R.SP;
    case OpAF: R.af8.FLAGS = (R.F.Z << 7) | (R.F.N << 6) | (R.F.H << 5) | (R.F.C << 4); return R.AF;
    case OpImm16: R.PC++; t = RB(rSystem, R.PC); R.PC++; t = t + ((RB(rSystem, R.PC) << 8)); return t;
    }
    return 0;
}

template<CPU::Operand16 op16>
void CPU::SetOperand16(const u16 v)
{
    switch (op16) {
    case OpBC: R.BC = v; break;
    case OpDE: R.DE = v; break;
    case OpHL: R.HL = v; break;
    case OpSP: R.SP = v; break;
    case OpAF: R.AF = v;
        R.F.Z = (R.af8.FLAGS & 0x80) ? 1 : 0;
        R.F.N = (R.af8.FLAGS & 0x40) ? 1 : 0;
        R.F.H = (R.af8.FLAGS & 0x20) ? 1 : 0;
        R.F.C = (R.af8.FLAGS & 0x10) ? 1 : 0;
        break;
    }
}

void CPU::NOP(System&) {
    R.PC++; 
}

template<CPU::Operand8 src>
void CPU::INC(System& rSystem) {
    u8 t = GetOperand8<src>(rSystem);
    R.F.H = ((t & 0xf) == 0xf) ? 1 : 0;
    t = (t + 1) & 0xff;
    SetOperand8<src>(rSystem, t);
    R.F.N = 0;
    R.F.Z = (t == 0) ? 1 : 0;
    R.PC++;
}

template<CPU::Operand16 src>
void CPU::INC16(System& rSystem) {
    u16 t = GetOperand16<src>(rSystem);
    t = t + 1;
    SetOperand16<src>(t);
    R.PC++;
}

template<CPU::Operand16 src>
void CPU::DEC16(System& rSystem) {
    u16 t = GetOperand16<src>(rSystem);
    t = t - 1;
    SetOperand16<src>(t);
    R.PC++; 
}

template<CPU::Operand8 src>
void CPU::DEC(System& rSystem)
{
    u8 t = GetOperand8<src>(rSystem);
    R.F.H = ((t & 0xf) == 0) ? 1 : 0;
    t = (t - 1) & 0xff;
    SetOperand8<src>(rSystem, t);
    R.F.N = 1;
    R.F.Z = (t == 0) ? 1 : 0;
    R.PC++;
}

template<CPU::Operand8 src, CPU::OpALU alu>
void CPU::ALU(System& rSystem)
{
#pragma warning( push )
#pragma warning( disable : 4127 )   // warning C4127: conditional expression is constant


    bool carry = (alu == ADC || alu == SBC);
    bool sub = !(alu == ADD || alu == ADC);
    size_t t = GetOperand8<src>(rSystem);

    if (sub) {
        R.F.H = (((R.af8.A & 0xf) - (t & 0xf) - (carry ? R.F.C : 0)) >> 4) & 1;
        t = R.af8.A - t - (carry ? R.F.C : 0);
    }
    else {
        R.F.H = ((t & 0xf) + (R.af8.A & 0xf) + (carry ? R.F.C : 0)) >> 4;
        t += R.af8.A + (carry ? R.F.C : 0);
    }
    if (alu != CP) {
        R.af8.A = t & 0xff;
    }
    R.F.Z = ((t & 0xff) == 0) ? 1 : 0;
    R.F.C = (t >> 8) & 1;
    R.F.N = sub ? 1 : 0;
    R.PC++;
#pragma warning( pop )
}

template<CPU::Operand16 src>
void CPU::ADDHL(System& rSystem) {
    size_t t = GetOperand16<src>(rSystem);
    R.F.H = ((R.HL & 0x0fff) + (t & 0x0fff) > 0x0fff) ? 1 : 0;
    t += R.HL;
    R.HL = (t & 0xffff);
    R.F.C = (t >> 16) & 1;
    R.F.N = 0;
    R.PC++;
    rSystem.Tick();
}

template<CPU::Operand8 src, CPU::OpBITW op>
void CPU::BITW(System& rSystem) {
    u8 t = GetOperand8<src>(rSystem);
    switch (op) {
    case AND: R.af8.A &= t; break;
    case OR:  R.af8.A |= t; break;
    case XOR:  R.af8.A ^= t; break;
    }
    R.F.C = 0;
    R.F.N = 0;
    R.F.H = (op == AND) ? 1 : 0;
    R.F.Z = (R.af8.A == 0) ? 1 : 0;
    R.PC++;
}

template<CPU::Operand8 src, CPU::OpBITS op>
void CPU::BITS(System& rSystem)
{
#pragma warning( push )
#pragma warning( disable : 4127 ) // warning C4127: conditional expression is constant
    bool z0 = (op == RLA || op == RLCA || op == RRA || op == RRCA);
    // TODO : Test with size_t
    u8 t = GetOperand8<src>(rSystem);
    u8 prevFC = R.F.C;
    if (op == RL || op == RLA || op == RLC || op == RLCA || op == SLA) {
        R.F.C = (t & 0x80) ? 1 : 0;
    }
    else if (op == RR || op == RRA || op == RRC || op == RRCA || op == SRA || op == SRL) {
        R.F.C = t & 1;
    }
    else { 
        R.F.C = 0; 
    }

    switch (op) {
    case RL:
    case RLA:  t = (t << 1) + prevFC; break;
    case RLC:
    case RLCA: t = (t << 1) + ((t >> 7) & 1); break;
    case RR:
    case RRA: t = (t >> 1) + (prevFC << 7); break;
    case RRC:
    case RRCA: t = (t >> 1) + ((t & 1) << 7); break;
    case SLA: t = t << 1; break;
    case SRA: t = (t >> 1) + (t & 0x80); break;
    case SRL: t = (t >> 1); break;
    case SWP: t = ((t & 0xf) << 4) + ((t & 0xf0) >> 4); break;
    }
    R.F.N = 0;
    R.F.H = 0;
    R.F.Z = (!z0 && (t & 0xff) == 0) ? 1 : 0;
    SetOperand8<src>(rSystem, t);
    R.PC++;
#pragma warning( pop )
}

template<CPU::Operand8 src, u8 bit>
void CPU::SET(System& rSystem) {
    SetOperand8<src>(rSystem, GetOperand8<src>(rSystem) | (1 << bit));
    R.PC++;
}

template<CPU::Operand8 src, u8 bit>
void CPU::RES(System& rSystem) {
    SetOperand8<src>(rSystem, GetOperand8<src>(rSystem) & (~(1 << bit)));
    R.PC++;
}

template<CPU::Operand8 src, u8 bit>
void CPU::BIT(System& rSystem) {
    R.F.N = 0;
    R.F.H = 1;
    R.F.Z = (GetOperand8<src>(rSystem) & (1 << bit)) ? 0 : 1;
    R.PC++;
}

template<CPU::Operand8 src, CPU::Operand8 dst>
void CPU::LD(System& rSystem) {
#pragma warning( push )
#pragma warning( disable : 4127 ) // warning C4127: conditional expression is constant
    if (src == OpIHL && dst == OpIHL) {
        HALT(rSystem); 
    } else {
        SetOperand8<dst>(rSystem, GetOperand8<src>(rSystem));
        R.PC++;
    }
#pragma warning( pop )
}

template<CPU::Operand16 src, CPU::Operand16 dst>
void CPU::LD16(System& rSystem) {
    SetOperand16<dst>(GetOperand16<src>(rSystem));
    R.PC++; 
}

void CPU::LDSP(System& rSystem) {
    u16 t = GetOperand16<OpImm16>(rSystem);
    WB(rSystem, t, R.SP & 0xff);
    t++;
    WB(rSystem, t, (R.SP >> 8) & 0xff);
    R.PC++; 
}

template<CPU::Operand16 dst>
void CPU::LDSPOF(System& rSystem)
{
    R.PC++;
    auto tu = RB(rSystem, R.PC);
    auto t = reinterpret_cast<signed char&>(tu);
    R.PC++;
    R.F.H = (((R.SP & 0x0f) + (tu & 0x0f)) >> 4) & 1;
    R.F.C = (((size_t)(R.SP & 0xff) + tu) >> 8) & 1;
    u16 t16 = R.SP + t;
    SetOperand16<dst>(t16);
    R.F.Z = 0;
    R.F.N = 0;
    rSystem.Tick();
}

void CPU::HALT(System&) {
    m_halt = true; R.PC++;
}

void CPU::STOP(System&) {
    R.PC++;
}

void CPU::CB(System& rSystem) {
    R.PC++; 
    (this->*(m_op_codes_CB[RB(rSystem, R.PC)]))(rSystem);
}

void CPU::SCF(System&) {
    R.F.C = 1;
    R.F.H = 0;
    R.F.N = 0;
    R.PC++;
}
void CPU::CCF(System&) {
    R.F.C = R.F.C ^ 1;
    R.F.H = 0;
    R.F.N = 0;
    R.PC++;
}

template<CPU::Operand16 src>
void CPU::PUSH(System& rSystem) {
    PUSH16(rSystem, GetOperand16<src>(rSystem));
    R.PC++;
}

template<CPU::Operand16 dst>
void CPU::POP(System& rSystem) {
    SetOperand16<dst>(POP16(rSystem));
    R.PC++;
}

template<CPU::CondFlag con>
void CPU::CALL(System& rSystem)
{
    u16 t = GetOperand16<OpImm16>(rSystem);
    R.PC++;
    if (Condition<con>()) {
        PUSH16(rSystem, R.PC);
        R.PC = t;
        rSystem.Tick(); 
    }
}

template<u8 addr>
void CPU::RST(System& rSystem) {
    R.PC++; PUSH16(rSystem, R.PC);
    R.PC = addr;
    rSystem.Tick();
}

template<CPU::CondFlag con>
void CPU::RET(System& rSystem) {
#pragma warning( push )
#pragma warning( disable : 4127 )   // warning C4127: conditional expression is constant

    if (Condition<con>()) {
        R.PC = POP16(rSystem);
        rSystem.Tick();
    }
    else {
        R.PC++;
    }
    if (con == RETI) {
        IME = true;
    }
#pragma warning( pop )
}

template<CPU::CondFlag con>
void CPU::JP(System& rSystem) {
    u16 t = GetOperand16<OpImm16>(rSystem);
    if (Condition<con>()) {
        R.PC = t;
        rSystem.Tick();
    }
    else { 
        R.PC++;
    }
}

template<CPU::CondFlag con>
void CPU::JR(System& rSystem) {
    R.PC++;
    u8 t = RB(rSystem, R.PC);
    R.PC++;
    if (Condition<con>()) {
        R.PC += reinterpret_cast<signed char&>(t);
        rSystem.Tick();
    }
}

void CPU::JPHL(System&) {
    R.PC = R.HL;
}

void CPU::EI(System&) {
    IME = true;
    R.PC++;
    m_ei_delay = true;
};

void CPU::DI(System&) {
    IME = false;
    R.PC++;
    m_ei_delay = false;
};

void CPU::CPL(System&) {
    R.af8.A = ~R.af8.A;
    R.F.N = 1;
    R.F.H = 1;
    R.PC++;
}

void CPU::DAA(System&)
{
    int tmp = R.af8.A;

    if (R.F.N) {
        if (R.F.H) {
            tmp -= 6;
            if (!(R.F.C)) {
                tmp &= 0xFF;
            }
        }
        if (R.F.C) {
            tmp -= 0x60;
        }
    }
    else {
        if ((R.F.H) || (tmp & 0x0F) > 9) {
            tmp += 6;
        }
        if ((R.F.C) || tmp > 0x9F) {
            tmp += 0x60;
        }
    }

    R.F.H = 0;
    if (tmp & 0x100) {
        R.F.C = 1;
    }
    R.af8.A = tmp & 0xFF;
    R.F.Z = (R.af8.A == 0) ? 1 : 0;

    R.PC++;

}

template<CPU::CondFlag con>
bool CPU::Condition() const {
    switch (con) {
    case CondZ: return (R.F.Z != 0);
    case CondNZ: return (R.F.Z == 0);
    case CondC: return (R.F.C != 0);
    case CondNC: return (R.F.C == 0);
    }
    return true;
}

void CPU::PUSH16(System& rSystem, const u16 v) {
    union { 
        struct {
            u8 L;
            u8 H;
        } hl8; u16 HL = 0;
    } t = {};
    
    t.HL = v;
    R.SP--;
    WB(rSystem, R.SP, t.hl8.H);
    R.SP--;
    WB(rSystem, R.SP, t.hl8.L);
}

u16 CPU::POP16(System& rSystem) {
    union {
        struct {
            u8 L;
            u8 H;
        } hl8; u16 HL = 0;
    } t = {};

    t.hl8.L = RB(rSystem, R.SP);
    R.SP++;
    t.hl8.H = RB(rSystem, R.SP);
    R.SP++;
    return t.HL;
}

#define STD_OPERANDS(x,y) x<OpB, y>, x<OpC, y>, x<OpD, y>, x<OpE, y>, x<OpH, y>, x<OpL, y>, x<OpIHL, y>, x<OpA, y>
const array<CPU::OpCodeFn, 256> CPU::m_op_codes = {
    &CPU::NOP,        &CPU::LD16<OpImm16, OpBC>,    &CPU::LD<OpA, OpIBC>, &CPU::INC16<OpBC>, &CPU::INC<OpB>,      &CPU::DEC<OpB>,   &CPU::LD<OpImm, OpB>,     &CPU::BITS<OpA, RLCA>,
    &CPU::LDSP,       &CPU::ADDHL<OpBC>,            &CPU::LD<OpIBC, OpA>, &CPU::DEC16<OpBC>, &CPU::INC<OpC>,      &CPU::DEC<OpC>,   &CPU::LD<OpImm, OpC>,     &CPU::BITS<OpA, RRCA>,
    &CPU::STOP,       &CPU::LD16<OpImm16, OpDE>,    &CPU::LD<OpA, OpIDE>, &CPU::INC16<OpDE>, &CPU::INC<OpD>,      &CPU::DEC<OpD>,   &CPU::LD<OpImm, OpD>,     &CPU::BITS<OpA, RLA>,
    &CPU::JR<Always>, &CPU::ADDHL<OpDE>,            &CPU::LD<OpIDE, OpA>, &CPU::DEC16<OpDE>, &CPU::INC<OpE>,      &CPU::DEC<OpE>,   &CPU::LD<OpImm, OpE>,     &CPU::BITS<OpA, RRA>,
    &CPU::JR<CondNZ>, &CPU::LD16<OpImm16, OpHL>,    &CPU::LD<OpA, OpHLI>, &CPU::INC16<OpHL>, &CPU::INC<OpH>,      &CPU::DEC<OpH>,   &CPU::LD<OpImm, OpH>,     &CPU::DAA,
    &CPU::JR<CondZ>,  &CPU::ADDHL<OpHL>,            &CPU::LD<OpHLI, OpA>, &CPU::DEC16<OpHL>, &CPU::INC<OpL>,      &CPU::DEC<OpL>,   &CPU::LD<OpImm, OpL>,     &CPU::CPL,
    &CPU::JR<CondNC>, &CPU::LD16<OpImm16, OpSP>,    &CPU::LD<OpA, OpHLD>, &CPU::INC16<OpSP>, &CPU::INC<OpIHL>,    &CPU::DEC<OpIHL>, &CPU::LD<OpImm, OpIHL>,   &CPU::SCF,
    &CPU::JR<CondC>,  &CPU::ADDHL<OpSP>,            &CPU::LD<OpHLD, OpA>, &CPU::DEC16<OpSP>, &CPU::INC<OpA>,      &CPU::DEC<OpA>,   &CPU::LD<OpImm, OpA>,     &CPU::CCF,
    STD_OPERANDS(&CPU::LD, OpB),
    STD_OPERANDS(&CPU::LD, OpC),
    STD_OPERANDS(&CPU::LD, OpD),
    STD_OPERANDS(&CPU::LD, OpE),
    STD_OPERANDS(&CPU::LD, OpH),
    STD_OPERANDS(&CPU::LD, OpL),
    STD_OPERANDS(&CPU::LD, OpIHL),
    STD_OPERANDS(&CPU::LD, OpA),
    STD_OPERANDS(&CPU::ALU, ADD),
    STD_OPERANDS(&CPU::ALU, ADC),
    STD_OPERANDS(&CPU::ALU, SUB),
    STD_OPERANDS(&CPU::ALU, SBC),
    STD_OPERANDS(&CPU::BITW, AND),
    STD_OPERANDS(&CPU::BITW, XOR),
    STD_OPERANDS(&CPU::BITW, OR),
    STD_OPERANDS(&CPU::ALU, CP),
    &CPU::RET<CondNZ>,        &CPU::POP<OpBC>,          &CPU::JP<CondNZ>,         &CPU::JP<Always>, &CPU::CALL<CondNZ>,  &CPU::PUSH<OpBC>,     &CPU::ALU<OpImm, ADD>,    &CPU::RST<0x00>,
    &CPU::RET<CondZ>,         &CPU::RET<Always>,        &CPU::JP<CondZ>,          &CPU::CB,         &CPU::CALL<CondZ>,   &CPU::CALL<Always>,   &CPU::ALU<OpImm, ADC>,    &CPU::RST<0x08>,
    &CPU::RET<CondNC>,        &CPU::POP<OpDE>,          &CPU::JP<CondNC>,         &CPU::NOP,        &CPU::CALL<CondNC>,  &CPU::PUSH<OpDE>,     &CPU::ALU<OpImm, SUB>,    &CPU::RST<0x10>,
    &CPU::RET<CondC>,         &CPU::RET<RETI>,          &CPU::JP<CondC>,          &CPU::NOP,        &CPU::CALL<CondC>,   &CPU::NOP,            &CPU::ALU<OpImm, SBC>,    &CPU::RST<0x18>,
    &CPU::LD<OpA, OpIoImm>,   &CPU::POP<OpHL>,          &CPU::LD<OpA, OpIoC>,     &CPU::NOP,        &CPU::NOP,           &CPU::PUSH<OpHL>,     &CPU::BITW<OpImm, AND>,   &CPU::RST<0x20>,
    &CPU::LDSPOF<OpSP>,       &CPU::JPHL,               &CPU::LD<OpA, OpIImm>,    &CPU::NOP,        &CPU::NOP,           &CPU::NOP,            &CPU::BITW<OpImm, XOR>,   &CPU::RST<0x28>,
    &CPU::LD<OpIoImm, OpA>,   &CPU::POP<OpAF>,          &CPU::LD<OpIoC, OpA>,     &CPU::DI,         &CPU::NOP,           &CPU::PUSH<OpAF>,     &CPU::BITW<OpImm, OR>,    &CPU::RST<0x30>,
    &CPU::LDSPOF<OpHL>,       &CPU::LD16<OpHL, OpSP>,   &CPU::LD<OpIImm, OpA>,    &CPU::EI,         &CPU::NOP,           &CPU::NOP,            &CPU::ALU<OpImm, CP>,     &CPU::RST<0x38>,
};

const array<CPU::OpCodeFn, 256> CPU::m_op_codes_CB = {
    STD_OPERANDS(&CPU::BITS, RLC), STD_OPERANDS(&CPU::BITS, RRC), STD_OPERANDS(&CPU::BITS, RL),  STD_OPERANDS(&CPU::BITS, RR),
    STD_OPERANDS(&CPU::BITS, SLA), STD_OPERANDS(&CPU::BITS, SRA), STD_OPERANDS(&CPU::BITS, SWP), STD_OPERANDS(&CPU::BITS, SRL),
    STD_OPERANDS(&CPU::BIT, 0),    STD_OPERANDS(&CPU::BIT, 1),    STD_OPERANDS(&CPU::BIT, 2),    STD_OPERANDS(&CPU::BIT, 3),
    STD_OPERANDS(&CPU::BIT, 4),    STD_OPERANDS(&CPU::BIT, 5),    STD_OPERANDS(&CPU::BIT, 6),    STD_OPERANDS(&CPU::BIT, 7),
    STD_OPERANDS(&CPU::RES, 0),    STD_OPERANDS(&CPU::RES, 1),    STD_OPERANDS(&CPU::RES, 2),    STD_OPERANDS(&CPU::RES, 3),
    STD_OPERANDS(&CPU::RES, 4),    STD_OPERANDS(&CPU::RES, 5),    STD_OPERANDS(&CPU::RES, 6),    STD_OPERANDS(&CPU::RES, 7),
    STD_OPERANDS(&CPU::SET, 0),    STD_OPERANDS(&CPU::SET, 1),    STD_OPERANDS(&CPU::SET, 2),    STD_OPERANDS(&CPU::SET, 3),
    STD_OPERANDS(&CPU::SET, 4),    STD_OPERANDS(&CPU::SET, 5),    STD_OPERANDS(&CPU::SET, 6),    STD_OPERANDS(&CPU::SET, 7),
};

bool GamePak::Load(const char * pRom_data, size_t size) {
    auto fnRead = [&](auto dest, size_t src_offset, size_t read_size) -> bool {
        if (src_offset + read_size <= size) {
            memcpy(dest, pRom_data + src_offset, read_size);
            return true;
        }
        else {
            return false;
        }
    };

    if (!fnRead(&m_header, 0x100, m_header.raw.size())) {
        printf("Error : Invalid header\n");
        return false;
    }

    char title[sizeof(m_header.fields.Title) + 1] = { 0 };

    memcpy(title, m_header.fields.Title, sizeof(m_header.fields.Title));

    printf("Title : %s\n", title);

    switch (m_header.fields.CartType) {
    case 0x00: m_MBC = MBC::None; break;
    case 0x01: case 0x02: case 0x03: m_MBC = MBC::MBC1;break;
    case 0x05: case 0x06: m_MBC = MBC::MBC2; break;
    case 0x0f: case 0x10: case 0x11: case 0x12: case 0x13: m_MBC = MBC::MBC3; break;
    case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: m_MBC = MBC::MBC5; break;
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
        size_t ram_size = max(m_RAM_size, size_t(8 * 1024));
        RAM.resize(ram_size);
        m_RAM_bank_count = ram_size / (8 * 1024);
        m_RAM_bank_mask = m_RAM_bank_count - 1;
        // TODO : fill ram with garbage
    }
    else
    {
        // Allocate some dummy ram space
        RAM.resize(8 * 1024);
        m_RAM_bank_mask = 0;
    }

    auto rom_size = (32 * 1024) << m_header.fields.ROMSize;
    ROM.resize(rom_size);

    if (!fnRead(ROM.data(), 0, rom_size)) {
        printf("Error : ROM too small#n");
        return false;
    }

    m_pCur_ROM_bank = &ROM[0x4000];
    m_ROM_bank_count = rom_size / (16 * 1024);
    m_ROM_bank_mask = m_ROM_bank_count - 1;

    return true;
}

template<Access eAccess>
u8 Video::VRAMAccess(const u16 addr, const u8 v) {
#pragma warning( push )
#pragma warning( disable : 4127 )   // warning C4127: conditional expression is constant

    if (eAccess == Access::Write) {
        VRAM[addr & 0x1fff] = v;
    } else {
        return VRAM[addr & 0x1fff];
    }
    return 0;
#pragma warning( pop )
}

template<Access eAccess>
u8 Video::OAMAccess(const u8 addr, const u8 v) {
#pragma warning( push )
#pragma warning( disable : 4127 )   // warning C4127: conditional expression is constant

    if (eAccess == Access::Write) {
        OAM[addr] = v;
    } else {
        return OAM[addr];
    }
    return 0;
#pragma warning( pop )
}

template <>
u8 GamePak::ROMAccess<Access::Write>(const u16 addr, const u8 v) {
    
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
            m_cur_ROM_bank += (m_MBC1_extra_select << 5);
        }
    };
    
    if (addr <= 0x1FFF) {
        m_ram_enabled = ((v & 0xF) == 0x0A);
    } else if (addr <= 0x3fff) {
        switch (m_MBC)
        {
        case MBC::None:
        default:
            break;
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
            m_cur_ROM_bank = m_ROM_bank_select;
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
            }
            else {
                m_MBC5_high_ROM_bank_select = v & 1;
            }
            m_cur_ROM_bank = m_ROM_bank_select;
            m_cur_ROM_bank += m_MBC5_high_ROM_bank_select << 8;
            fnUpdateCurROMBank();
            break;
        }
    } else if (addr <= 0x5fff) {
        switch (m_MBC)
        {
        case MBC::None:
        case MBC::MBC2:
        default:
            break;
        case MBC::MBC1:
            m_MBC1_extra_select = v & 0x03;
            if (m_MBC1_extra_mode == MBC1ExtraMode::ROM)
            {
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
                m_cur_RAM_bank = m_RAM_bank_select;
                fnUpdateCurRAMBank();
            }
            break;
        case MBC::MBC5:
            m_RAM_bank_select = v & 0x03;
            m_cur_RAM_bank = m_RAM_bank_select;
            fnUpdateCurRAMBank();
            break;
        }
    } else if (addr <= 0x7fff)
    {
        if (m_MBC == MBC::MBC1)
        {
            m_MBC1_extra_mode = (v & 1) ? MBC1ExtraMode::RAM : MBC1ExtraMode::ROM;
            fnComputeMBC1ROMBank();
            fnUpdateCurROMBank();
            m_cur_RAM_bank = (v & 1) ? m_MBC1_extra_select : 0;
            fnUpdateCurRAMBank();
        }
    }

    return 0;
}

template <>
u8 GamePak::ROMAccess<Access::Read>(const u16 addr, const u8) {
    if (addr < 0x4000) {
        return ROM[addr];
    }
    else {
        return m_pCur_ROM_bank[addr & 0x3fff];
    }
}

template<Access eAccess>
u8 GamePak::RAMAccess(const u16 addr, const u8 v)
{
#pragma warning( push )
#pragma warning( disable : 4127 )   // warning C4127: conditional expression is constant

    if (m_ram_enabled)
    {
        if (eAccess == Access::Write) {
            RAM[addr & 0x1fff] = v;
        }
        else {
            return RAM[addr & 0x1fff];
        }
    }

    return 0xFF;
#pragma warning( pop )
}

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
            SB = v;
            if ((v & 0x80) && (v & 1)) {
                m_serial_bits = 8;
            }
        } else {
            return 0xfe | SC;
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
        if (eAccess == Access::Write) {
            rSystem.m_video.LY = 0;
        } else {
            return rSystem.m_video.LY;
        }
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
    }

    return 0;

#pragma warning( pop )
}

void CPU::DoOAMDMA(System& rSystem) {

    if (m_oam_dma_active) {

        u16 addr = m_oam_dma_src + m_oam_dma_index;
        rSystem.m_video.OAM[m_oam_dma_index] = rSystem.BusAccess<Access::Read>(addr);
        ++m_oam_dma_index;
        if (m_oam_dma_index == 160) {
            m_oam_dma_active = false;
        }
    }
}

void CPU::Execute(System& rSystem)
{
    if (m_halt) {
        rSystem.Tick();
    } else {
        auto opcode = RB(rSystem, R.PC);
        (this->*m_op_codes[opcode])(rSystem);
    }

    if (m_ei_delay) {
        m_ei_delay = false;
    } else if (IF.value & IE.value & 0x1f) {
        m_halt = false;
        if (IME) {
            u16 ivector = 0x40;
            if (IF.f.joypad & IE.f.joypad){
                IF.f.joypad = 0;
                ivector = 0x60;
            } else if (IF.f.serial & IE.f.serial) {
                IF.f.serial = 0;
                ivector = 0x58;
            } else if (IF.f.timer & IE.f.timer) {
                IF.f.timer = 0;
                ivector = 0x50;
            } else if (IF.f.lcdstat & IE.f.lcdstat) {
                IF.f.lcdstat = 0;
                ivector = 0x48;
            } else if (IF.f.vblank & IE.f.vblank) {
                IF.f.vblank = 0;
                ivector = 0x40;
            };
            IME = false;
            PUSH16(rSystem, R.PC);
            R.PC = ivector;
            rSystem.Tick();
        }
    }
}

void IO::Reset()
{
    Divider.value = 0;
    TIMA = 0;
    TMA = 0;
    TAC = 0;
    m_timer_mask = 255;
    P1 = 0x0f;
    m_keys.value = 0xff;
}

void IO::Tick(System& m_rSystem)
{
    Divider.value += 4;

    if (TAC & 4) {
        if (((u8)m_rSystem.m_cycle_count & m_timer_mask) == 0)
        if (TIMA == 255) {
            TIMA = TMA;
            m_rSystem.m_cpu.IF.f.timer = 1;
        } else {
            ++TIMA;
        }
    }

    if (m_serial_bits) {
        --m_serial_bits;
        SB = SB << 1 | 1;
        if (!m_serial_bits) {
            m_rSystem.m_cpu.IF.f.serial = 1;
        }
    }
}

u8 IO::MakeP1(System& rSystem)
{
    u8 OldP1 = P1;
    u8 v = 0xf;
    if (P1 & 0x20) {v = v & m_keys.value;}
    if (P1 & 0x10) {v = v & (m_keys.value >> 4);}
    P1 = (P1 & 0x30) | v | 0xc0;
    if (OldP1 & (~P1) & 0xf)
    {
        rSystem.m_cpu.IF.f.joypad = 1;
    }
    return P1;
}

void IO::SetTAC(u8 v)
{
    TAC = v;
    switch (v & 3) {
    case 0 : m_timer_mask = 255; break;
    case 1 : m_timer_mask = 3; break;
    case 2 : m_timer_mask = 15; break;
    case 3 : m_timer_mask = 63; break;
    }
}

void Video::Reset()
{
    m_upFront_buf.reset(new FrameBuf);
    m_upBack_buf.reset(new FrameBuf);

    STAT.value = 0x84;
    m_mode = 2;
    LCDX = 0;

    LCDC.value = 0x91;
    LY = 0;
    SCX = 0;
    SCY = 0;
    WX = 0;
    WY = 0;
    BGP = 0xFC;
    OBP0 = 0xff;
    OBP1 = 0xff;
}

void Video::Tick(System & rSystem)
{
    LCDX++;
    switch(m_mode)
    {
    case 0 : // Mode 0 H-Blank
        if (LCDX == 114) {
            LCDX = 0; 
            LY++;
            if (LCDC.bits.lcden && LY == LYC && STAT.bits.coincidenceint) {
                rSystem.m_cpu.IF.f.lcdstat = 1;
            }
            if (LY == 144) {
                m_mode = 1; // Transition to Mode 1 VBlank
                Flip(rSystem);
                if (LCDC.bits.lcden) {
                    rSystem.m_cpu.IF.f.vblank = 1;
                    if (STAT.bits.mode1int) {
                        rSystem.m_cpu.IF.f.lcdstat = 1;
                    }
                }
            } else { 
                m_mode = 2;
            }
        } 
        break;
    case 1: // Mode 1 V-Blank
        if (LCDX == 114) {
            LCDX = 0;
            LY++;
            if (LY == 154) {
                LY = 0;
                m_mode = 2; // Transition to Mode 2 OAM  
                if (LCDC.bits.lcden && STAT.bits.mode2int) {
                    rSystem.m_cpu.IF.f.lcdstat = 1;
                }
            }
            if (LCDC.bits.lcden && LY == LYC && STAT.bits.coincidenceint) {
                rSystem.m_cpu.IF.f.lcdstat = 1;
            }
        } 
        break;
    case 2: // Mode 2 OAM
        if (LCDX == 23) {
            m_mode = 3;    // Transition to Mode 3 OAM+VRAM
            RenderLine();
        } 
        break;
    case 3: // Mode 3 OAM+VRAM
        if (LCDX == 63) {
            m_mode = 0;   // Transition to Mode 0 HBlank
            if (LCDC.bits.lcden && STAT.bits.mode0int) {
                rSystem.m_cpu.IF.f.lcdstat = 1;
            }
        } 
        break;
    }
}

void Video::Flip(System & rSystem)
{
    swap(m_upBack_buf, m_upFront_buf);

    rSystem.m_new_frame = true;
    ++rSystem.m_frame_number;
}

void Video::RenderLine()
{
    size_t render_y = LY;
    if (render_y >= 144) {
        return;
    }

    auto fnGetTile = [this](const u8 tile, const u8 yofs, const u8* vramdata) {
        u16 tiledata = (tile << 4) + (yofs << 1);
        u8 datah = vramdata[tiledata]; u8 datal = vramdata[tiledata + 1];
        u16 data = ((datal & 0x01) << 14)| ((datah & 0x01) << 15)| ((datal & 0x02) << 11)| ((datah & 0x02) << 12)|
            ((datal & 0x04) << 8) | ((datah & 0x04) << 9) | ((datal & 0x08) << 5) | ((datah & 0x08) << 6) |
            ((datal & 0x10) << 2) | ((datah & 0x10) << 3) | ((datal & 0x20) >> 1) | ((datah & 0x20)     ) |
            ((datal & 0x40) >> 4) | ((datah & 0x40) >> 3) | ((datal & 0x80) >> 7) | ((datah & 0x80) >> 6);
        data = ((data & 0x5555) << 1) | ((data & 0xaaaa) >> 1);
        return data;
    };

    auto fnGetRGB = [](u8 pix) -> u32 {return  0xFF000000 | ((3 - pix) * 0x00555555);};
    
    /*static constexpr array<u32, 4> zero_pal = {
        0xFF000000,
        0xFF555555,
        0xFFBBBBBB,
        0xFFFFFFFF,
    };

    static constexpr array<u32, 4> bg_pal = {
        0xFF220022,
        0xFF552055,
        0xFF993099,
        0xFFEE50EE,
    };

    static constexpr array<u32, 4> win_pal = {
        0xFF102222,
        0xFF305555,
        0xFF509999,
        0xFF70EEEE,
    };
    static constexpr array<u32, 4> spr_pal = {
        0xFF222210,
        0xFF555530,
        0xFF999950,
        0xFFEEEE70,
    };
    auto fnGetRGB = [](u8 pix, const array<u32,4>& pal) -> u32 {
        return  pal[3 - pix];
    };
*/
    u32* line = &m_upFront_buf->pix[render_y * 160];

    const u8 bgcol[4] = {
        static_cast<u8>(BGP & 0x3),
        static_cast<u8>((BGP >> 2) & 3),
        static_cast<u8>((BGP >> 4) & 3),
        static_cast<u8>((BGP >> 6) & 3)};

    //auto bgfillcol = fnGetRGB(bgcol[0], zero_pal);
    auto bgfillcol = fnGetRGB(bgcol[0]);
    fill_n(line, 160, bgfillcol);

    if (!LCDC.bits.lcden)
    {
        return;
    }

    u8 sprcache[40];
    u8 sprcacheidx = 0;
    u8 lastsprcacheidx = 0;
    u8 sprsize = LCDC.bits.objsize ? 16 : 8;

    // Find sprites
    if (LCDC.bits.objdisplay)
    {
        for (int spridx = 0; spridx < 160; spridx+=4)
        {
            int y = OAM[spridx];
            if ((y <= LY + 16) && (y + sprsize > LY + 16))
            {
                u8 attrib = OAM[spridx + 3];
                u8 yofs = static_cast<u8>((LY + 16) - y);
                if (attrib & 0x40) {
                    yofs = sprsize - 1 - yofs;
                }
                u8 tile;
                if (LCDC.bits.objsize) {
                    tile = (OAM[spridx + 2] & 0xfe) | ((yofs & 0x08) >> 3);
                } else {
                    tile = OAM[spridx + 2];
                }

                u16 data = fnGetTile(tile, yofs & 7, &VRAM[0]);
                if (attrib & 0x20) {
                    data = ((data & 0x3333) << 2) | ((data & 0xcccc) >> 2);
                    data = ((data & 0x0f0f) << 4) | ((data & 0xf0f0) >> 4);
                    data = ((data & 0x00ff) << 8) | ((data & 0xff00) >> 8);
                }
                sprcache[sprcacheidx] = OAM[spridx + 1];
                sprcacheidx++;
                sprcache[sprcacheidx] = attrib;
                sprcacheidx++;
                *(u16*)(&sprcache[sprcacheidx]) = data;
                sprcacheidx+=2;
                lastsprcacheidx = sprcacheidx;
                if (sprcacheidx == 40) break;
            }
        }
    }

    enum RenderStep
    {
        RenderStepSpritesBehind = 0,
        RenderStepBKG = 1,
        RenderStepSpritesOver = 2,
        RenderStepEnd = 3,
    };

    for (auto step = RenderStep(0); step < RenderStepEnd; step = RenderStep(step + 1))
    {
        if (step == RenderStepSpritesBehind || step == RenderStepSpritesOver)
        {
            const u8 objcol0[4] = {
                static_cast<u8>(OBP0 & 0x3),
                static_cast<u8>((OBP0 >> 2) & 3),
                static_cast<u8>((OBP0 >> 4) & 3),
                static_cast<u8>((OBP0 >> 6) & 3)};

            const u8 objcol1[4] = {
                static_cast<u8>(OBP1 & 0x3),
                static_cast<u8>((OBP1 >> 2) & 3),
                static_cast<u8>((OBP1 >> 4) & 3),
                static_cast<u8>((OBP1 >> 6) & 3)};

            // Sprite
            for (u8 spridx = 0; spridx < sprcacheidx; spridx+=4)
            {
                u8 attrib = sprcache[spridx+1];
                if ((step == RenderStepSpritesBehind && (attrib & 0x80)) ||
                    (step == RenderStepSpritesOver   && !(attrib & 0x80)))
                {
                    u8 sprx = sprcache[spridx];
                    if (sprx < 168)
                    {
                        u16 data = *(u16*)(&sprcache[spridx + 2]);
                        for (u8 x = sprx; x < sprx + 8;x++)
                        {
                            u8 pixel = data & 0x3;
                            if (pixel && x >= 8 && x < 168) {
                                line[x - 8] = fnGetRGB((attrib & 0x10) ? objcol1[pixel] : objcol0[pixel]);
                                //line[x - 8] = fnGetRGB((attrib & 0x10) ? objcol1[pixel] : objcol0[pixel], spr_pal);
                            }
                            data >>=2;
                        }
                    }
                }
            }
        }
        else
        {
            const u8* bgData = &VRAM[LCDC.bits.bgwintiledata ? 0 : 0x800];
            u8 bgAdd = LCDC.bits.bgwintiledata ? 0 : 128;
            u8 bgwinx = SCX & 0x7;
            u16 tileaddr = LCDC.bits.bgtilemap ? 0x1c00 : 0x1800;
            tileaddr += ((((LY + SCY) & 0xff) >> 3) << 5) + (SCX >> 3);
            u16 bgwindata = 0;
            u8 bgyofs = (LY + SCY) & 0x7;
            bool bWinOn = LCDC.bits.windisp && WY <= LY;
            for (u8 x = 0; x < 168; x++)
            {
                if (bWinOn && x == WX )
                {
                    tileaddr = LCDC.bits.wintilemap ? 0x1c00 : 0x1800;
                    tileaddr += (((LY - WY) >> 3) << 5);
                    bgyofs = (LY - WY) & 0x7;
                    bgwinx = 7;
                }

                u8 pixel = bgwindata & 3;
                if (pixel > 0 && x >= 8) {
                    line[x - 8] = fnGetRGB(bgcol[pixel]);
                    //line[x - 8] = fnGetRGB(bgcol[pixel], (bWinOn && x >= WX) ? win_pal : bg_pal);
                }

                bgwinx++;
                if (bgwinx == 8 && LCDC.bits.bgdisplay)
                {
                    u8 tile = VRAM[tileaddr];
                    tileaddr = (tileaddr & 0xffe0) | ((tileaddr+1) & 0x1f);
                    tile+=bgAdd;
                    bgwindata = fnGetTile(tile, bgyofs, bgData);
                    bgwinx = 0;
                }
                else
                {
                    bgwindata >>=2;
                }

            }
        }
    }
}

void System::Reset()
{
    m_cpu.Reset();
    m_video.Reset();
    m_io.Reset();
    m_audio.Reset();
    m_frame_number = 0;
    m_cycle_count = 0;
    m_new_frame = false;
}

void System::RunFrame()
{
    m_new_frame = false;
    while (!m_new_frame)
    {
        m_cpu.Execute(*this);
    }
}

nanoseconds System::RunTime(nanoseconds time_to_run)
{
    u64 cycles_to_run = time_to_run.count() * 1024LL * 1024LL / 1000000000LL;
    auto start_cycle_count = m_cycle_count;
    auto end_cycle_count = start_cycle_count + cycles_to_run;

    while (m_cycle_count < end_cycle_count)
    {
        m_cpu.Execute(*this);
    }

    return nanoseconds((m_cycle_count - start_cycle_count) * 1000000000LL / (1024LL * 1024LL));
}

void Audio::Reset()
{
    m_sq1.Reset(*this);
    m_sq2.Reset(*this);
    m_wave.Reset(*this);
    m_noise.Reset(*this);

    // Boot rom state
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
    m_sq1.m_active = false;
    m_sq2.m_active = false;
    m_wave.m_active = false;
    m_noise.m_active = false;

    m_left = 0;
    m_right = 0;
}

void Audio::Tick(System &)
{
    if (m_master_enable)
    {
        // Advance sequencer
        if ((m_audio_clock & 2047) == 0)
        {
            // Advance lenght
            if ((m_sequencer_clock & 1) == 0) {
                m_sq1.RunLenght();
                m_sq2.RunLenght();
                m_wave.RunLenght();
                m_noise.RunLenght();
            }

            // Advance sweep
            if (m_sequencer_clock == 2 || m_sequencer_clock == 6)
            {
                m_sq1.RunSweep();
            }

            // Advance enveloppe
            if (m_sequencer_clock == 7)
            {
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
        i16 left = 0;
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

        rSample.right = m_left * 2;
        rSample.left = m_right * 2;

        m_left = 0;
        m_right = 0;

        // Mix some right to left and left to rightfor confort
        //sample.right = left + right / 4;
        //sample.left = right + left / 4;
    }
}

template<>
u8 Audio::RegAccess<Access::Read>(const u8 addr, const u8)
{
    if (addr >= 0x30 && addr < 0x40)
    {
        if (m_wave.m_active) {
            return m_wave.m_cur_wave;
        } else {
            return AUD3WAVERAM[addr - 0x30];
        }
    }

    switch (addr)
    {
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
            return 0xF0 |
                (m_sq1.m_active ? 0x01 : 0) |
                (m_sq2.m_active ? 0x02 : 0) |
                (m_wave.m_active ? 0x04 : 0) |
                (m_noise.m_active ? 0x08 : 0);
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
    if (addr >= 0x30 && addr < 0x40)
    {
        AUD3WAVERAM[addr - 0x30] = v;
        return 0xff;
    }

    if (!m_master_enable && addr !=0x26) {
        return 0xff;
    }

    switch (addr)
    {
    case 0x10:
        NR10 = v;
        m_sq1.m_sweep.freq = (v >> 4) & 0x7;
        m_sq1.m_sweep.inc = (v & 8) == 0;
        m_sq1.m_sweep.shift = v & 7;
        break;
    case 0x11:
        NR11 = v;
        m_sq1.m_sq.duty = v >> 6;
        m_sq1.m_lenght = 64 - (int)(v & 63);
        break;
    case 0x12:
        NR12 = v;
        m_sq1.m_env.init_vol = v >> 4;
        m_sq1.m_env.inc = (v & 0x08) != 0;
        m_sq1.m_env.freq = v & 7;
        m_sq1.m_dac_active = (v & 0xF8) != 0;
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
        m_sq1.m_freq = (m_sq1.m_freq & 0xff) | ((v & 7) << 8);
        m_sq1.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80) {
            m_sq1.m_active = m_sq1.m_env.init_vol || m_sq1.m_env.inc;
            m_sq1.m_period = 2048 - m_sq1.m_freq;
            m_sq1.m_env.period = m_sq1.m_env.freq;
            m_sq1.m_env.vol = m_sq1.m_env.init_vol;
            if (m_sq1.m_sweep.freq || m_sq1.m_sweep.shift) {
                m_sq1.m_sweep.period = m_sq1.m_sweep.freq;
                m_sq1.m_sweep.freq_int = m_sq1.m_freq;
                m_sq1.m_sweep.active = true;
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
        m_sq2.m_lenght = 64 - (int)(v & 63);
        break;
    case 0x17:
        NR22 = v;
        m_sq2.m_env.init_vol = v >> 4;
        m_sq2.m_env.inc = (v & 0x08) != 0;
        m_sq2.m_env.freq = v & 7;
        m_sq2.m_dac_active = (v & 0xF8) != 0;
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
        m_sq2.m_freq = (m_sq2.m_freq & 0xff) | ((v & 7) << 8);
        m_sq2.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80) {
            m_sq2.m_active = m_sq2.m_env.init_vol || m_sq2.m_env.inc;
            m_sq2.m_period = 2048 - m_sq2.m_freq;
            m_sq2.m_env.period = m_sq2.m_env.freq;
            m_sq2.m_env.vol = m_sq2.m_env.init_vol;
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
        NR31 = v;   // TODO : could be removed
        m_wave.m_lenght = 256 - (int)v;
        break;
    case 0x1c:
        NR32 = v;
        switch ((v >> 5) & 3)
        {
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
        m_wave.m_freq = (m_wave.m_freq & 0xff) | ((v & 7) << 8);
        m_wave.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80)
        {
            m_wave.m_active = m_wave.m_dac_active;
            m_wave.m_period = 2048 - m_wave.m_freq;
            m_wave.m_phase = 0;
        }
        break;
    case 0x20:
        NR41 = v;
        m_noise.m_lenght = 64 - (int)(v & 63);
        break;
    case 0x21:
        NR42 = v;
        m_noise.m_env.init_vol = v >> 4;
        m_noise.m_env.inc = (v & 0x08) != 0;
        m_noise.m_env.freq = v & 7;
        m_noise.m_dac_active = (v & 0xF8) != 0;
        if (!m_noise.m_dac_active) {
            m_noise.m_active = false;
        }
        break;
    case 0x22:
        NR43 = v;
        m_noise.m_freq = (v & 0xF0) >> 4;
        m_noise.m_use7bits = (v & 8) != 0;
        m_noise.m_div = (v & 7) << 2;
        if (m_noise.m_div == 0) {
            m_noise.m_div = 2;
        }
        m_noise.m_period = (int)m_noise.m_div << (int)m_noise.m_freq;
        break;
    case 0x23:
        NR44 = v;
        m_noise.m_use_lenght = (v & 0x40) != 0;
        if (v & 0x80) {
            m_noise.m_active = m_noise.m_env.init_vol || m_noise.m_env.inc;
            m_noise.m_env.period = m_noise.m_env.freq;
            m_noise.m_env.vol = m_noise.m_env.init_vol;
            m_noise.m_lfsr = 0xFFFF;
        }
        break;
    case 0x24:
        NR50 = v;
        m_right_vol = ((v & 0x70) >> 4) + 1;
        m_left_vol = (v & 7) + 1;
        break;
    case 0x25:
        NR51 = v;
        break;
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

void SoundChannel::Reset(Audio & rAudio)
{
    m_active = false;
    m_dac_active = false;
    m_freq = 0;
    m_lenght = 0;
    m_use_lenght = true;
    m_out_val = 0;
    m_cur_wave = 0;
    m_period = 0;

    rAudio.NR50 = 0;
    rAudio.NR51 = 0;
}

void SoundChannel::RunLenght()
{
    if (m_active && m_use_lenght) {
        if (m_lenght > 0){
            --m_lenght;
            if (m_lenght == 0)
            {
                m_active = false;
            }
        }
    }
}

void SoundChannel::RunEnv(SoundEnv & rEnv)
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
                } if (rEnv.vol > 0) {
                    rEnv.period = rEnv.freq;
                } else {
                    rEnv.period = 0;
                    m_active = false;
                }
            }
        }
    }
}

void SoundChannel::RunSquare(SquareWave & rSq)
{
    if (!m_active) {
        return;
    }

    if (m_period == 0) {
        bool out = false;
        rSq.phase = (rSq.phase + 1) & 7;
        switch(rSq.duty)
        {
        case 0: out = rSq.phase == 7; break;
        case 1: out = rSq.phase == 0 || rSq.phase == 7; break;
        case 2: out = rSq.phase == 0 || rSq.phase >= 5; break;
        case 3: out = rSq.phase != 0 && rSq.phase != 7; break;
        }
        m_out_val = out ? 1 : 0;
        m_period = 2048 - m_freq;
    } else {
        --m_period;
    }
}

void Square1::Reset(Audio & rAudio)
{
    SoundChannel::Reset(rAudio);

    m_sweep.active = false;
    m_sweep.freq = 0;
    m_sweep.freq_int = 0;
    m_sweep.inc = false;
    m_sweep.period = 0;
    m_sweep.shift = 0;
    m_sq.duty = 0;
    m_sq.phase = 0;

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
    int adjust = (int)m_sweep.freq_int >> m_sweep.shift;
    if (m_sweep.inc) {
        int new_freq = (int)m_freq + adjust;
        if (new_freq >= 0x800) {
            m_sweep.active = false;
            m_active = false;
        } else if (write_freq_reg && m_sweep.shift){
            m_freq = (u16)(new_freq & 0x7FF);
            m_period = 2048 - m_freq;
        }
    } else {
        int new_freq = (int)m_freq - adjust;
        if (new_freq < 0) {
            m_sweep.active = false;
            m_active = false;
        } else if (write_freq_reg && m_sweep.shift){
            m_freq = (u16)(new_freq & 0x7FF);
            m_period = 2048 - m_freq;
        }
    }
}

void Square2::Reset(Audio & rAudio)
{
    SoundChannel::Reset(rAudio);

    m_sq.duty = 0;
    m_sq.phase = 0;

    rAudio.NR21 = 0;
    rAudio.NR22 = 0;
    rAudio.NR23 = 0;
    rAudio.NR24 = 0;
}

void SoundWave::Reset(Audio & rAudio)
{
    SoundChannel::Reset(rAudio);

    m_vol_shift = 0;
    m_phase = 0;
    m_cur_wave = 0;
    m_cur_val = 0;

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
        m_cur_val = m_cur_wave;
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

void Noise::Reset(Audio & rAudio)
{
    SoundChannel::Reset(rAudio);

    m_use7bits = false;
    m_div = 0;
    m_lfsr = 0xff;

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
                m_lfsr = (m_lfsr << 1) | (((m_lfsr & 0x40) >> 6) ^ ((m_lfsr & 0x20) >> 5));
            } else {
                m_out_val = (((m_lfsr & 0x4000) >> 14) ^ 1);
                m_lfsr = (m_lfsr << 1) | (((m_lfsr & 0x4000) >> 14) ^ ((m_lfsr & 0x2000) >> 13));
            }
        }
    } else {
        --m_period;
    }
}

} // namespce GB

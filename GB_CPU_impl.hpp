#include "GB_CPU.h"
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

void CPU::Reset(ResetOption reset_opt)
{
    if (reset_opt & ResetOption_USE_BOOT_ROM) {
        R.PC        = 0;
        R.SP        = 0;
        R.af8.A     = 0;
        R.af8.FLAGS = 0;
        R.F.Z       = 0;
        R.F.N       = 0;
        R.F.C       = 0;
        R.F.H       = 0;
        R.BC        = 0;
        R.DE        = 0;
        R.HL        = 0;
        IF.value    = 0xE1;
    } else {
        R.PC        = 0x100;
        R.SP        = 0xFFFE;
        R.af8.A     = 0x01;
        R.af8.FLAGS = 0xB0;
        R.F.Z       = 1;
        R.F.N       = 0;
        R.F.C       = 1;
        R.F.H       = 1;
        R.BC        = 0x0013;
        R.DE        = 0x00d8;
        R.HL        = 0x014d;
        IF.value    = 0;
    }

    fill(begin(HRAM), end(HRAM), u8(0));

    // bQuit = false;
    m_halt           = false;
    m_ei_delay       = false;
    m_oam_dma_active = false;
    m_oam_dma_src    = 0;
    m_oam_dma_index  = 0;
}

inline u8 CPU::RB(System& rSystem, const u16 addr)
{
    rSystem.Tick();
    return rSystem.BusAccess<Access::Read>(addr);
}

inline void CPU::WB(System& rSystem, const u16 addr, const u8 v)
{
    rSystem.Tick();
    rSystem.BusAccess<Access::Write>(addr, v);
}

// clang-format off
template<> u16 CPU::GetOperand16<CPU::OpBC>(System&) {return R.BC;}
template<> u16 CPU::GetOperand16<CPU::OpDE>(System&) {return R.DE;}
template<> u16 CPU::GetOperand16<CPU::OpHL>(System&) {return R.HL;}
template<> u16 CPU::GetOperand16<CPU::OpSP>(System&) {return R.SP;}
template<> u16 CPU::GetOperand16<CPU::OpAF>(System&) {R.af8.FLAGS = (R.F.Z << 7) | (R.F.N << 6) | (R.F.H << 5) | (R.F.C << 4); return R.AF;}
template<> u16 CPU::GetOperand16<CPU::OpImm16>(System& rSystem) {R.PC++; u16 t = RB(rSystem, R.PC); R.PC++; t = t + ((RB(rSystem, R.PC) << 8)); return t;}

template<> u8 CPU::GetOperand8<CPU::OpA>(System&) {return R.af8.A;}
template<> u8 CPU::GetOperand8<CPU::OpB>(System&) {return R.bc8.B;}
template<> u8 CPU::GetOperand8<CPU::OpC>(System&) {return R.bc8.C;}
template<> u8 CPU::GetOperand8<CPU::OpD>(System&) {return R.de8.D;}
template<> u8 CPU::GetOperand8<CPU::OpE>(System&) {return R.de8.E;}
template<> u8 CPU::GetOperand8<CPU::OpH>(System&) {return R.hl8.H;}
template<> u8 CPU::GetOperand8<CPU::OpL>(System&) {return R.hl8.L;}
template<> u8 CPU::GetOperand8<CPU::OpIHL>(System& rSystem) {return RB(rSystem, R.HL);}
template<> u8 CPU::GetOperand8<CPU::OpIBC>(System& rSystem) {return RB(rSystem, R.BC);}
template<> u8 CPU::GetOperand8<CPU::OpIDE>(System& rSystem) {return RB(rSystem, R.DE);}
template<> u8 CPU::GetOperand8<CPU::OpImm>(System& rSystem) {R.PC++; return RB(rSystem, R.PC);}
template<> u8 CPU::GetOperand8<CPU::OpIoImm>(System& rSystem) {R.PC++; const u8 t = RB(rSystem, R.PC); return RB(rSystem, 0xff00 | t);}
template<> u8 CPU::GetOperand8<CPU::OpIoC>(System& rSystem) {return RB(rSystem, 0xff00 |  R.bc8.C);}
template<> u8 CPU::GetOperand8<CPU::OpHLI>(System& rSystem) {const u8 t = RB(rSystem, R.HL); R.HL++; return t;}
template<> u8 CPU::GetOperand8<CPU::OpHLD>(System& rSystem) {const u8 t = RB(rSystem, R.HL); R.HL--; return t;}
template<> u8 CPU::GetOperand8<CPU::OpIImm>(System& rSystem) {const u16 t = GetOperand16<OpImm16>(rSystem); return RB(rSystem, t);}

template<> void CPU::SetOperand16<CPU::OpBC>(const u16 v) {R.BC = v;}
template<> void CPU::SetOperand16<CPU::OpDE>(const u16 v) {R.DE = v;}
template<> void CPU::SetOperand16<CPU::OpHL>(const u16 v) {R.HL = v;}
template<> void CPU::SetOperand16<CPU::OpSP>(const u16 v) {R.SP = v;}
template<> void CPU::SetOperand16<CPU::OpAF>(const u16 v) {
    R.AF = v;
    R.F.Z = (R.af8.FLAGS & 0x80) ? 1 : 0;
    R.F.N = (R.af8.FLAGS & 0x40) ? 1 : 0;
    R.F.H = (R.af8.FLAGS & 0x20) ? 1 : 0;
    R.F.C = (R.af8.FLAGS & 0x10) ? 1 : 0;
}

template<> void CPU::SetOperand8<CPU::OpA>(System&, const u8 v) {R.af8.A = v;}
template<> void CPU::SetOperand8<CPU::OpB>(System&, const u8 v) {R.bc8.B = v;}
template<> void CPU::SetOperand8<CPU::OpC>(System&, const u8 v) {R.bc8.C = v;}
template<> void CPU::SetOperand8<CPU::OpD>(System&, const u8 v) {R.de8.D = v;}
template<> void CPU::SetOperand8<CPU::OpE>(System&, const u8 v) {R.de8.E = v;}
template<> void CPU::SetOperand8<CPU::OpH>(System&, const u8 v) {R.hl8.H = v;}
template<> void CPU::SetOperand8<CPU::OpL>(System&, const u8 v) {R.hl8.L = v;}
template<> void CPU::SetOperand8<CPU::OpIHL>(System& rSystem, const u8 v) {WB(rSystem, R.HL, v);}
template<> void CPU::SetOperand8<CPU::OpIBC>(System& rSystem, const u8 v) {WB(rSystem, R.BC, v);}
template<> void CPU::SetOperand8<CPU::OpIDE>(System& rSystem, const u8 v) {WB(rSystem, R.DE, v);}
template<> void CPU::SetOperand8<CPU::OpIoImm>(System& rSystem, const u8 v) {R.PC++; const u8 t = RB(rSystem, R.PC); WB(rSystem, 0xff00 | t, v);}
template<> void CPU::SetOperand8<CPU::OpIoC>(System& rSystem, const u8 v) {WB(rSystem, 0xff00 |  R.bc8.C, v);}
template<> void CPU::SetOperand8<CPU::OpHLI>(System& rSystem, const u8 v) {WB(rSystem, R.HL, v); R.HL++;}
template<> void CPU::SetOperand8<CPU::OpHLD>(System& rSystem, const u8 v) {WB(rSystem, R.HL, v); R.HL--;}
template<> void CPU::SetOperand8<CPU::OpIImm>(System& rSystem, const u8 v) {const u16 t = GetOperand16<OpImm16>(rSystem); WB(rSystem, t, v);}

template<> bool CPU::CheckCondition<CPU::CondAlways>() const {return true;}
template<> bool CPU::CheckCondition<CPU::CondZ>()      const {return R.F.Z != 0;}
template<> bool CPU::CheckCondition<CPU::CondNZ>()     const {return R.F.Z == 0;}
template<> bool CPU::CheckCondition<CPU::CondC>()      const {return R.F.C != 0;}
template<> bool CPU::CheckCondition<CPU::CondNC>()     const {return R.F.C == 0;}
template<> bool CPU::CheckCondition<CPU::CondRETI>()   const {return true;}

// clang-format on

void CPU::NOP(System&)
{
    R.PC++;
}

template<class TOp8>
void CPU::INC(System& rSystem)
{
    AssertOp8<TOp8>();

    u8 t  = GetOperand8<TOp8>(rSystem);
    R.F.H = ((t & 0xf) == 0xf) ? 1 : 0;
    t     = (t + 1) & 0xff;
    SetOperand8<TOp8>(rSystem, t);
    R.F.N = 0;
    R.F.Z = (t == 0) ? 1 : 0;
    R.PC++;
}

template<class TOp16>
void CPU::INC16(System& rSystem)
{
    AssertOp16<TOp16>();

    u16 t = GetOperand16<TOp16>(rSystem);
    t     = t + 1;
    SetOperand16<TOp16>(t);
    R.PC++;
    rSystem.Tick();
}

template<class TOp16>
void CPU::DEC16(System& rSystem)
{
    AssertOp16<TOp16>();

    u16 t = GetOperand16<TOp16>(rSystem);
    t     = t - 1;
    SetOperand16<TOp16>(t);
    R.PC++;
    rSystem.Tick();
}

template<class TOp8>
void CPU::DEC(System& rSystem)
{
    AssertOp8<TOp8>();

    u8 t  = GetOperand8<TOp8>(rSystem);
    R.F.H = ((t & 0xf) == 0) ? 1 : 0;
    t     = (t - 1) & 0xff;
    SetOperand8<TOp8>(rSystem, t);
    R.F.N = 1;
    R.F.Z = (t == 0) ? 1 : 0;
    R.PC++;
}

template<class TOp8, CPU::OpALU alu>
void CPU::ALU(System& rSystem)
{
    AssertOp8<TOp8>();

    constexpr bool carry = (alu == ADC || alu == SBC);
    constexpr bool sub   = !(alu == ADD || alu == ADC);

    size_t t = GetOperand8<TOp8>(rSystem);

    if constexpr (sub) {
        R.F.H = (((R.af8.A & 0xf) - (t & 0xf) - (carry ? R.F.C : 0)) >> 4) & 1;
        t     = R.af8.A - t - (carry ? R.F.C : 0);
    } else {
        R.F.H = ((t & 0xf) + (R.af8.A & 0xf) + (carry ? R.F.C : 0)) >> 4;
        t += R.af8.A + (carry ? R.F.C : 0);
    }
    if constexpr (alu != CP) {
        R.af8.A = t & 0xff;
    }
    R.F.Z = ((t & 0xff) == 0) ? 1 : 0;
    R.F.C = (t >> 8) & 1;
    R.F.N = sub ? 1 : 0;
    R.PC++;
}

template<class TOp16>
void CPU::ADDHL(System& rSystem)
{
    AssertOp16<TOp16>();

    size_t t = GetOperand16<TOp16>(rSystem);
    R.F.H    = ((R.HL & 0x0fff) + (t & 0x0fff) > 0x0fff) ? 1 : 0;
    t += R.HL;
    R.HL  = (t & 0xffff);
    R.F.C = (t >> 16) & 1;
    R.F.N = 0;
    R.PC++;
    rSystem.Tick();
}

template<class TOp8, CPU::OpBITW op>
void CPU::BITW(System& rSystem)
{
    AssertOp8<TOp8>();

    u8 t = GetOperand8<TOp8>(rSystem);
    switch (op) {
    case AND: R.af8.A &= t; break;
    case OR: R.af8.A |= t; break;
    case XOR: R.af8.A ^= t; break;
    }
    R.F.C = 0;
    R.F.N = 0;
    R.F.H = (op == AND) ? 1 : 0;
    R.F.Z = (R.af8.A == 0) ? 1 : 0;
    R.PC++;
}

template<class TOp8, CPU::OpBITS op>
void CPU::BITS(System& rSystem)
{
    AssertOp8<TOp8>();

    constexpr bool z0 = (op == RLA || op == RLCA || op == RRA || op == RRCA);

    // TODO : Test with size_t if it generates better code
    u8 t      = GetOperand8<TOp8>(rSystem);
    u8 prevFC = R.F.C;
    if constexpr (op == RL || op == RLA || op == RLC || op == RLCA || op == SLA) {
        R.F.C = (t & 0x80) ? 1 : 0;
    } else if constexpr (op == RR || op == RRA || op == RRC || op == RRCA || op == SRA || op == SRL) {
        R.F.C = t & 1;
    } else {
        R.F.C = 0;
    }

    switch (op) {
    case RL:
    case RLA: t = (t << 1) + prevFC; break;
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
    SetOperand8<TOp8>(rSystem, t);
    R.PC++;
}

template<class TOp8, u8 bit>
void CPU::SET(System& rSystem)
{
    AssertOp8<TOp8>();

    SetOperand8<TOp8>(rSystem, GetOperand8<TOp8>(rSystem) | (1 << bit));
    R.PC++;
}

template<class TOp8, u8 bit>
void CPU::RES(System& rSystem)
{
    AssertOp8<TOp8>();

    SetOperand8<TOp8>(rSystem, GetOperand8<TOp8>(rSystem) & (~(1 << bit)));
    R.PC++;
}

template<class TOp8, u8 bit>
void CPU::BIT(System& rSystem)
{
    AssertOp8<TOp8>();

    R.F.N = 0;
    R.F.H = 1;
    R.F.Z = (GetOperand8<TOp8>(rSystem) & (1 << bit)) ? 0 : 1;
    R.PC++;
}

template<>
void CPU::LD<CPU::OpIHL, CPU::OpIHL>(System& rSystem)
{
    HALT(rSystem);
}

template<class TOpSrc8, class TOpDst8>
void CPU::LD(System& rSystem)
{
    AssertOp8<TOpSrc8>();
    AssertOp8<TOpDst8>();

    SetOperand8<TOpDst8>(rSystem, GetOperand8<TOpSrc8>(rSystem));
    R.PC++;
}

template<class Op16DstT>
void CPU::LD16(System& rSystem)
{
    AssertOp16<Op16DstT>();

    SetOperand16<Op16DstT>(GetOperand16<OpImm16>(rSystem));
    R.PC++;
}

void CPU::LDSP(System& rSystem)
{
    u16 t = GetOperand16<OpImm16>(rSystem);
    WB(rSystem, t, R.SP & 0xff);
    t++;
    WB(rSystem, t, (R.SP >> 8) & 0xff);
    R.PC++;
}

void CPU::LDSPHL(System& rSystem)
{
    R.SP = R.HL;
    rSystem.Tick();
    R.PC++;
}

template<class TOp16>
void CPU::LDSPOF(System& rSystem)
{
    AssertOp16<TOp16>();

    R.PC++;
    const u8   tu = RB(rSystem, R.PC);
    const auto t  = reinterpret_cast<const signed char&>(tu);

    R.PC++;
    R.F.H = (((R.SP & 0x0f) + (tu & 0x0f)) >> 4) & 1;
    R.F.C = (((size_t)(R.SP & 0xff) + tu) >> 8) & 1;

    const u16 t16 = R.SP + t;
    SetOperand16<TOp16>(t16);
    R.F.Z = 0;
    R.F.N = 0;
    rSystem.Tick();
    if constexpr (TOp16::op16 == Op16::SP) {
        rSystem.Tick();
    }
}

void CPU::HALT(System&)
{
    m_halt = true;
    R.PC++;
}

void CPU::STOP(System&)
{
    R.PC++;
}

void CPU::CB(System& rSystem)
{
    R.PC++;
    (this->*(m_op_codes_CB[RB(rSystem, R.PC)]))(rSystem);
}

void CPU::SCF(System&)
{
    R.F.C = 1;
    R.F.H = 0;
    R.F.N = 0;
    R.PC++;
}
void CPU::CCF(System&)
{
    R.F.C = R.F.C ^ 1;
    R.F.H = 0;
    R.F.N = 0;
    R.PC++;
}

template<class TOp16>
void CPU::PUSH(System& rSystem)
{
    AssertOp16<TOp16>();

    PUSH16(rSystem, GetOperand16<TOp16>(rSystem));
    R.PC++;
}

template<class TOp16>
void CPU::POP(System& rSystem)
{
    AssertOp16<TOp16>();

    SetOperand16<TOp16>(POP16(rSystem));
    R.PC++;
}

template<class TCond>
void CPU::CALL(System& rSystem)
{
    u16 t = GetOperand16<OpImm16>(rSystem);
    R.PC++;
    if (CheckCondition<TCond>()) {
        PUSH16(rSystem, R.PC);
        R.PC = t;
    }
}

template<u8 addr>
void CPU::RST(System& rSystem)
{
    R.PC++;
    PUSH16(rSystem, R.PC);
    R.PC = addr;
}

template<class TCond>
void CPU::RET(System& rSystem)
{
    rSystem.Tick();
    if (CheckCondition<TCond>()) {
        R.PC = POP16(rSystem);
        if constexpr (TCond::cond != Condition::Always && TCond::cond != Condition::RETI) {
            rSystem.Tick();
        }
    } else {
        R.PC++;
    }
    if constexpr (TCond::cond == Condition::RETI) {
        IME = true;
    }
}

template<class TCond>
void CPU::JP(System& rSystem)
{
    const u16 t = GetOperand16<OpImm16>(rSystem);
    if (CheckCondition<TCond>()) {
        R.PC = t;
        rSystem.Tick();
    } else {
        R.PC++;
    }
}

template<class TCond>
void CPU::JR(System& rSystem)
{
    R.PC++;
    const u8 t = RB(rSystem, R.PC);
    R.PC++;
    if (CheckCondition<TCond>()) {
        R.PC += reinterpret_cast<const signed char&>(t);
        rSystem.Tick();
    }
}

void CPU::JPHL(System&)
{
    R.PC = R.HL;
}

void CPU::EI(System&)
{
    IME = true;
    R.PC++;
    m_ei_delay = true;
};

void CPU::DI(System&)
{
    IME = false;
    R.PC++;
    m_ei_delay = false;
};

void CPU::CPL(System&)
{
    R.af8.A = ~R.af8.A;
    R.F.N   = 1;
    R.F.H   = 1;
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
    } else {
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
    R.F.Z   = (R.af8.A == 0) ? 1 : 0;

    R.PC++;
}



void CPU::PUSH16(System& rSystem, const u16 v)
{
    union {
        struct {
            u8 L;
            u8 H;
        } hl8;
        u16 HL = 0;
    } t = {};

    t.HL = v;
    R.SP--;
    WB(rSystem, R.SP, t.hl8.H);
    R.SP--;
    WB(rSystem, R.SP, t.hl8.L);
    rSystem.Tick();
}

u16 CPU::POP16(System& rSystem)
{
    union {
        struct {
            u8 L;
            u8 H;
        } hl8;
        u16 HL = 0;
    } t = {};

    t.hl8.L = RB(rSystem, R.SP);
    R.SP++;
    t.hl8.H = RB(rSystem, R.SP);
    R.SP++;
    return t.HL;
}

// clang-format off
#define STD_OPERANDS(x,y) x<OpB, y>, x<OpC, y>, x<OpD, y>, x<OpE, y>, x<OpH, y>, x<OpL, y>, x<OpIHL, y>, x<OpA, y>
const array<CPU::OpCodeFn, 256> CPU::m_op_codes = {
    &CPU::NOP,              &CPU::LD16<OpBC>,  &CPU::LD<OpA, OpIBC>,  &CPU::INC16<OpBC>, &CPU::INC<OpB>,      &CPU::DEC<OpB>,    &CPU::LD<OpImm, OpB>,     &CPU::BITS<OpA, RLCA>,
    &CPU::LDSP,             &CPU::ADDHL<OpBC>, &CPU::LD<OpIBC, OpA>,  &CPU::DEC16<OpBC>, &CPU::INC<OpC>,      &CPU::DEC<OpC>,    &CPU::LD<OpImm, OpC>,     &CPU::BITS<OpA, RRCA>,
    &CPU::STOP,             &CPU::LD16<OpDE>,  &CPU::LD<OpA, OpIDE>,  &CPU::INC16<OpDE>, &CPU::INC<OpD>,      &CPU::DEC<OpD>,    &CPU::LD<OpImm, OpD>,     &CPU::BITS<OpA, RLA>,
    &CPU::JR<CondAlways>,   &CPU::ADDHL<OpDE>, &CPU::LD<OpIDE, OpA>,  &CPU::DEC16<OpDE>, &CPU::INC<OpE>,      &CPU::DEC<OpE>,    &CPU::LD<OpImm, OpE>,     &CPU::BITS<OpA, RRA>,
    &CPU::JR<CondNZ>,       &CPU::LD16<OpHL>,  &CPU::LD<OpA, OpHLI>,  &CPU::INC16<OpHL>, &CPU::INC<OpH>,      &CPU::DEC<OpH>,    &CPU::LD<OpImm, OpH>,     &CPU::DAA,
    &CPU::JR<CondZ>,        &CPU::ADDHL<OpHL>, &CPU::LD<OpHLI, OpA>,  &CPU::DEC16<OpHL>, &CPU::INC<OpL>,      &CPU::DEC<OpL>,    &CPU::LD<OpImm, OpL>,     &CPU::CPL,
    &CPU::JR<CondNC>,       &CPU::LD16<OpSP>,  &CPU::LD<OpA, OpHLD>,  &CPU::INC16<OpSP>, &CPU::INC<OpIHL>,    &CPU::DEC<OpIHL>,  &CPU::LD<OpImm, OpIHL>,   &CPU::SCF,
    &CPU::JR<CondC>,        &CPU::ADDHL<OpSP>, &CPU::LD<OpHLD, OpA>,  &CPU::DEC16<OpSP>, &CPU::INC<OpA>,      &CPU::DEC<OpA>,    &CPU::LD<OpImm, OpA>,     &CPU::CCF,
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
    &CPU::RET<CondNZ>,      &CPU::POP<OpBC>,       &CPU::JP<CondNZ>,      &CPU::JP<CondAlways>, &CPU::CALL<CondNZ>,  &CPU::PUSH<OpBC>,       &CPU::ALU<OpImm, ADD>,    &CPU::RST<0x00>,
    &CPU::RET<CondZ>,       &CPU::RET<CondAlways>, &CPU::JP<CondZ>,       &CPU::CB,             &CPU::CALL<CondZ>,   &CPU::CALL<CondAlways>, &CPU::ALU<OpImm, ADC>,    &CPU::RST<0x08>,
    &CPU::RET<CondNC>,      &CPU::POP<OpDE>,       &CPU::JP<CondNC>,      &CPU::NOP,            &CPU::CALL<CondNC>,  &CPU::PUSH<OpDE>,       &CPU::ALU<OpImm, SUB>,    &CPU::RST<0x10>,
    &CPU::RET<CondC>,       &CPU::RET<CondRETI>,   &CPU::JP<CondC>,       &CPU::NOP,            &CPU::CALL<CondC>,   &CPU::NOP,              &CPU::ALU<OpImm, SBC>,    &CPU::RST<0x18>,
    &CPU::LD<OpA, OpIoImm>, &CPU::POP<OpHL>,       &CPU::LD<OpA, OpIoC>,  &CPU::NOP,            &CPU::NOP,           &CPU::PUSH<OpHL>,       &CPU::BITW<OpImm, AND>,   &CPU::RST<0x20>,
    &CPU::LDSPOF<OpSP>,     &CPU::JPHL,            &CPU::LD<OpA, OpIImm>, &CPU::NOP,            &CPU::NOP,           &CPU::NOP,              &CPU::BITW<OpImm, XOR>,   &CPU::RST<0x28>,
    &CPU::LD<OpIoImm, OpA>, &CPU::POP<OpAF>,       &CPU::LD<OpIoC, OpA>,  &CPU::DI,             &CPU::NOP,           &CPU::PUSH<OpAF>,       &CPU::BITW<OpImm, OR>,    &CPU::RST<0x30>,
    &CPU::LDSPOF<OpHL>,     &CPU::LDSPHL,          &CPU::LD<OpIImm, OpA>, &CPU::EI,             &CPU::NOP,           &CPU::NOP,              &CPU::ALU<OpImm, CP>,     &CPU::RST<0x38>,
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
// clang-format on

void CPU::DoOAMDMA(System& rSystem)
{

    if (m_oam_dma_active) {

        const u16 addr                       = m_oam_dma_src + m_oam_dma_index;
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
        const u8 opcode = RB(rSystem, R.PC);
        (this->*m_op_codes[opcode])(rSystem);
    }

    if (m_ei_delay) {
        m_ei_delay = false;
    } else if (IF.value & IE.value & 0x1f) {
        if (IME) {
            u16 ivector = 0x40;
            if (IF.f.joypad & IE.f.joypad) {
                IF.f.joypad = 0;
                ivector     = 0x60;
            } else if (IF.f.serial & IE.f.serial) {
                IF.f.serial = 0;
                ivector     = 0x58;
            } else if (IF.f.timer & IE.f.timer) {
                IF.f.timer = 0;
                ivector    = 0x50;
            } else if (IF.f.lcdstat & IE.f.lcdstat) {
                IF.f.lcdstat = 0;
                ivector      = 0x48;
            } else if (IF.f.vblank & IE.f.vblank) {
                IF.f.vblank = 0;
                ivector     = 0x40;
            };
            IME = false;
            PUSH16(rSystem, R.PC);
            R.PC   = ivector;
            m_halt = false;
        } else if (m_halt) {
            // HALT bug
            const u8 opcode = RB(rSystem, R.PC);
            --R.PC; // HALT bug executes an instruction wihtout increasing PC
            (this->*m_op_codes[opcode])(rSystem);
            m_halt = false;
        }
    }
}

} // namespace GB
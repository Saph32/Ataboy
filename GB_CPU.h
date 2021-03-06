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

class CPU {
public:
    enum class Op8 {
        A,     // Register A
        B,     // Register B
        C,     // Register C
        D,     // Register D
        E,     // Register E
        H,     // Register H
        L,     // Register L
        IHL,   // Indirect (HL)
        Imm,   // Immediate 16 bits
        IBC,   // Indirect (BC)
        IDE,   // Indirect (DE)
        IoImm, // Indirect IO (0xFF00 + Immediate 8 bits)
        IoC,   // Indirect IO (0xFF00 + C)
        HLI,   // Indirect (HL), post increment
        HLD,   // Indirect (HL), post decrement
        IImm,  // Indirect (Immediate 16 bits)
    };

    struct Op8Type {
    };

#define DEFINE_OP8_TYPE(x)                                                                                   \
    struct Op##x : public Op8Type {                                                                          \
        static constexpr auto op8 = Op8::x;                                                                  \
    };

    DEFINE_OP8_TYPE(A);
    DEFINE_OP8_TYPE(B);
    DEFINE_OP8_TYPE(C);
    DEFINE_OP8_TYPE(D);
    DEFINE_OP8_TYPE(E);
    DEFINE_OP8_TYPE(H);
    DEFINE_OP8_TYPE(L);
    DEFINE_OP8_TYPE(IHL);
    DEFINE_OP8_TYPE(Imm);
    DEFINE_OP8_TYPE(IBC);
    DEFINE_OP8_TYPE(IDE);
    DEFINE_OP8_TYPE(IoImm);
    DEFINE_OP8_TYPE(IoC);
    DEFINE_OP8_TYPE(HLI);
    DEFINE_OP8_TYPE(HLD);
    DEFINE_OP8_TYPE(IImm);

    template<class T>
    constexpr static void AssertOp8() {
        static_assert(std::is_base_of<Op8Type, T>::value, "Parameter type is not Op8.");
    }

    enum class Op16 {
        BC,    // Register BC
        DE,    // Register DE
        HL,    // Register HL
        SP,    // Register SP
        AF,    // Register AF
        Imm16, // Immediate 16 bits
    };
    struct Op16Type {
    };

#define DEFINE_OP16_TYPE(x)                                                                                  \
    struct Op##x : public Op16Type {                                                                         \
        static constexpr auto op16 = Op16::x;                                                                \
    };

    DEFINE_OP16_TYPE(BC);
    DEFINE_OP16_TYPE(DE);
    DEFINE_OP16_TYPE(HL);
    DEFINE_OP16_TYPE(SP);
    DEFINE_OP16_TYPE(AF);
    DEFINE_OP16_TYPE(Imm16);

    template<class T>
    constexpr static void AssertOp16() {
        static_assert(std::is_base_of<Op16Type, T>::value, "Parameter type is not Op16.");
    }

    enum OpALU { ADD, ADC, SUB, SBC, CP };
    enum OpBITW { AND, OR, XOR };
    enum OpBITS { RL, RLA, RLC, RLCA, RR, RRA, RRC, RRCA, SLA, SRA, SRL, SWP };

    enum class Condition {
        Always, // Always execute
        Z,      // Execute if zero flag = 1
        NZ,     // Execute if zero flag = 0
        C,      // Execute if condition flag = 1
        NC,     // Execute if consition flag = 0
        RETI    // Execute from a RETI instruction
    };

    struct CondType {
    };

#define DEFINE_COND_TYPE(x)                                                                                  \
    struct Cond##x : public CondType {                                                                       \
        static constexpr auto cond = Condition::x;                                                                \
    };

    DEFINE_COND_TYPE(Always);
    DEFINE_COND_TYPE(Z);
    DEFINE_COND_TYPE(NZ);
    DEFINE_COND_TYPE(C);
    DEFINE_COND_TYPE(NC);
    DEFINE_COND_TYPE(RETI);

    typedef void (CPU::*OpCodeFn)(System&);

    // Registers
    // clang-format off
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
    // clang-format on

    std::array<u8, 128> HRAM = {};

    // Internal state
    bool IME = false; // Interrupt enable

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
    IFLAG  IF = {};

    bool m_halt           = false;
    bool m_ei_delay       = false;
    bool m_oam_dma_active = false;
    u16  m_oam_dma_src    = 0;
    u8   m_oam_dma_index  = 0;

    static const std::array<OpCodeFn, 256> m_op_codes;
    static const std::array<OpCodeFn, 256> m_op_codes_CB;

    void Reset(ResetOption reset_opt);

    u8   RB(System& rSystem, const u16 addr);
    void WB(System& rSystem, const u16 addr, const u8 v);

    // clang-format off
    template<class Op8T>  u8   GetOperand8(System& rSystem);
    template<class Op8T>  void SetOperand8(System& rSystem, const u8 v);
    template<class Op16T> u16  GetOperand16(System& rSystem);
    template<class Op16T> void SetOperand16(const u16 v);

    template<class CondT> bool CheckCondition() const;

    void NOP(System& rSystem);

    template<class Op8T>                   void INC(System& rSystem);
    template<class Op16T>                  void INC16(System& rSystem);
    template<class Op16T>                  void DEC16(System& rSystem);
    template<class Op8T>                   void DEC(System& rSystem);
    template<class Op8T, OpALU alu>        void ALU(System& rSystem);
    template<class Op16T>                  void ADDHL(System& rSystem);
    template<class Op8T, OpBITW op>        void BITW(System& rSystem);
    template<class Op8T, OpBITS op>        void BITS(System& rSystem);
    template<class Op8T, u8 bit>           void SET(System& rSystem);
    template<class Op8T, u8 bit>           void RES(System& rSystem);
    template<class Op8T, u8 bit>           void BIT(System& rSystem);
    template<class Op8SrcT, class Op8DstT> void LD(System& rSystem);
    template<class Op16DstT>               void LD16(System& rSystem);
    template<class Op16T>                  void LDSPOF(System& rSystem);

    void LDSP(System& rSystem);
    void LDSPHL(System& rSystem);
    void HALT(System& rSystem);
    void STOP(System& rSystem);
    void CB(System& rSystem);
    void SCF(System& rSystem);
    void CCF(System& rSystem);

    template<class Op16T>        void PUSH(System& rSystem);
    template<class Op16T>        void POP(System& rSystem);
    template<class CondType>     void CALL(System& rSystem);
    template<u8 addr>            void RST(System& rSystem);
    template<class CondType>     void RET(System& rSystem);
    template<class CondType>     void JP(System& rSystem);
    template<class CondType>     void JR(System& rSystem);

    void JPHL(System& rSystem);
    void EI(System& rSystem);
    void DI(System& rSystem);
    void CPL(System& rSystem);
    void DAA(System& rSystem);
    void PUSH16(System& rSystem, const u16 v);
    u16  POP16(System& rSystem);
    // clang-format on

    void DoOAMDMA(System& rSystem);
    void Execute(System& rSystem);
};
} // namespace GB
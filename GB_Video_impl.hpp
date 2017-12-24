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

void Video::Reset(ResetOption reset_opt)
{
    m_upFront_buf.reset(new FrameBuf);
    m_upBack_buf.reset(new FrameBuf);

    if (reset_opt & ResetOption_USE_BOOT_ROM) {
        LCDC.value = 0;
        m_mode     = 0;
    } else {
        LCDC.value = 0x91;
        m_mode     = 2;
    }

    STAT.value = 0x84;
    LCDX       = 0;

    LY   = 0;
    SCX  = 0;
    SCY  = 0;
    WX   = 0;
    WY   = 0;
    BGP  = 0xFC;
    OBP0 = 0xff;
    OBP1 = 0xff;
}

void Video::Tick(System& rSystem)
{
    LCDX++;
    switch (m_mode) {
    case 0: // Mode 0 H-Blank
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
                LY     = 0;
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
            m_mode = 3; // Transition to Mode 3 OAM+VRAM
            RenderLine();
        }
        break;
    case 3: // Mode 3 OAM+VRAM
        if (LCDX == 63) {
            m_mode = 0; // Transition to Mode 0 HBlank
            if (LCDC.bits.lcden && STAT.bits.mode0int) {
                rSystem.m_cpu.IF.f.lcdstat = 1;
            }
        }
        break;
    }
}

void Video::Flip(System& rSystem)
{
    swap(m_upBack_buf, m_upFront_buf);

    rSystem.m_new_frame = true;
    ++rSystem.m_frame_number;
}

void Video::RenderLine()
{
    const size_t render_y = LY;
    if (render_y >= 144) {
        return;
    }

    auto fnGetTile = [](const u8 tile, const u8 yofs, const u8* vramdata) {
        const u16 tiledata = (tile << 4) + (yofs << 1);
        const u8  datah    = vramdata[tiledata];
        u8        datal    = vramdata[tiledata + 1];
        u16       data     = ((datal & 0x01) << 14) | ((datah & 0x01) << 15) | ((datal & 0x02) << 11) |
                   ((datah & 0x02) << 12) | ((datal & 0x04) << 8) | ((datah & 0x04) << 9) |
                   ((datal & 0x08) << 5) | ((datah & 0x08) << 6) | ((datal & 0x10) << 2) |
                   ((datah & 0x10) << 3) | ((datal & 0x20) >> 1) | ((datah & 0x20)) | ((datal & 0x40) >> 4) |
                   ((datah & 0x40) >> 3) | ((datal & 0x80) >> 7) | ((datah & 0x80) >> 6);
        data = ((data & 0x5555) << 1) | ((data & 0xaaaa) >> 1);
        return data;
    };

    auto fnGetRGB = [](u8 pix) -> u32 { return 0xFF000000 | ((3 - pix) * 0x00555555); };

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
    u32* const line = &m_upFront_buf->pix[render_y * 160];

    const u8 bgcol[4] = {static_cast<u8>(BGP & 0x3),
                         static_cast<u8>((BGP >> 2) & 3),
                         static_cast<u8>((BGP >> 4) & 3),
                         static_cast<u8>((BGP >> 6) & 3)};

    // auto bgfillcol = fnGetRGB(bgcol[0], zero_pal);
    u32 bgfillcol = fnGetRGB(bgcol[0]);
    fill_n(line, 160, bgfillcol);

    if (!LCDC.bits.lcden) {
        return;
    }

    u8 sprcache[40];
    u8 sprcacheidx     = 0;
    u8 lastsprcacheidx = 0;

    const u8 sprsize = LCDC.bits.objsize ? 16 : 8;

    // Find sprites
    if (LCDC.bits.objdisplay) {
        for (int spridx = 0; spridx < 160; spridx += 4) {
            int y = OAM[spridx];
            if ((y <= LY + 16) && (y + sprsize > LY + 16)) {
                u8 attrib = OAM[spridx + 3];
                u8 yofs   = static_cast<u8>((LY + 16) - y);
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
                sprcacheidx += 2;
                lastsprcacheidx = sprcacheidx;
                if (sprcacheidx == 40)
                    break;
            }
        }
    }

    enum RenderStep {
        RenderStepSpritesBehind = 0,
        RenderStepBKG           = 1,
        RenderStepSpritesOver   = 2,
        RenderStepEnd           = 3,
    };

    for (auto step = RenderStep(0); step < RenderStepEnd; step = RenderStep(step + 1)) {
        if (step == RenderStepSpritesBehind || step == RenderStepSpritesOver) {
            const u8 objcol0[4] = {static_cast<u8>(OBP0 & 0x3),
                                   static_cast<u8>((OBP0 >> 2) & 3),
                                   static_cast<u8>((OBP0 >> 4) & 3),
                                   static_cast<u8>((OBP0 >> 6) & 3)};

            const u8 objcol1[4] = {static_cast<u8>(OBP1 & 0x3),
                                   static_cast<u8>((OBP1 >> 2) & 3),
                                   static_cast<u8>((OBP1 >> 4) & 3),
                                   static_cast<u8>((OBP1 >> 6) & 3)};

            // Sprite
            for (u8 idx = 0; idx < sprcacheidx; idx += 4) {
                const u8 spridx = sprcacheidx - idx - 4;    // Lower index have higher priority
                const u8 attrib = sprcache[spridx + 1];
                if ((step == RenderStepSpritesBehind && (attrib & 0x80)) ||
                    (step == RenderStepSpritesOver && !(attrib & 0x80))) {
                    const u8 sprx = sprcache[spridx];
                    if (sprx < 168) {
                        u16 data = *(u16*)(&sprcache[spridx + 2]);
                        for (u8 x = sprx; x < sprx + 8; x++) {
                            u8 pixel = data & 0x3;
                            if (pixel && x >= 8 && x < 168) {
                                line[x - 8] = fnGetRGB((attrib & 0x10) ? objcol1[pixel] : objcol0[pixel]);
                                // line[x - 8] = fnGetRGB((attrib & 0x10) ? objcol1[pixel] : objcol0[pixel],
                                // spr_pal);
                            }
                            data >>= 2;
                        }
                    }
                }
            }
        } else {
            const u8* const bgData   = &VRAM[LCDC.bits.bgwintiledata ? 0 : 0x800];
            const u8        bgAdd    = LCDC.bits.bgwintiledata ? 0 : 128;
            u8              bgwinx   = SCX & 0x7;
            u16             tileaddr = LCDC.bits.bgtilemap ? 0x1c00 : 0x1800;
            tileaddr += ((((LY + SCY) & 0xff) >> 3) << 5) + (SCX >> 3);
            u16  bgwindata = 0;
            u8   bgyofs    = (LY + SCY) & 0x7;
            bool bWinOn    = LCDC.bits.windisp && WY <= LY;
            for (u8 x = 0; x < 168; x++) {
                if (bWinOn && x == WX) {
                    tileaddr = LCDC.bits.wintilemap ? 0x1c00 : 0x1800;
                    tileaddr += (((LY - WY) >> 3) << 5);
                    bgyofs = (LY - WY) & 0x7;
                    bgwinx = 7;
                }

                const u8 pixel = bgwindata & 3;
                if (pixel > 0 && x >= 8) {
                    line[x - 8] = fnGetRGB(bgcol[pixel]);
                    // line[x - 8] = fnGetRGB(bgcol[pixel], (bWinOn && x >= WX) ? win_pal : bg_pal);
                }

                bgwinx++;
                if (bgwinx == 8 && LCDC.bits.bgdisplay) {
                    u8 tile  = VRAM[tileaddr];
                    tileaddr = (tileaddr & 0xffe0) | ((tileaddr + 1) & 0x1f);
                    tile += bgAdd;
                    bgwindata = fnGetTile(tile, bgyofs, bgData);
                    bgwinx    = 0;
                } else {
                    bgwindata >>= 2;
                }
            }
        }
    }
}

template<>
u8 Video::VRAMAccess<Access::Read>(const u16 addr, const u8)
{
    return VRAM[addr & 0x1fff];
}

template<>
u8 Video::VRAMAccess<Access::Write>(const u16 addr, const u8 v)
{
    VRAM[addr & 0x1fff] = v;
    return 0;
}

template<>
u8 Video::OAMAccess<Access::Read>(const u8 addr, const u8 )
{
    return OAM[addr];
}

template<>
u8 Video::OAMAccess<Access::Write>(const u8 addr, const u8 v)
{
    OAM[addr] = v;
    return 0;
}

} // namespace GB

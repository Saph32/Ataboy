// This is an open source non-commercial project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

// Copyright (c) 2016-2017, Jean Charles Vallieres
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

#include <vector>
#include <fstream>
#include <cstdio>
#include <array>
#include <thread>
#include <chrono>
#include <mutex>

#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

#include "FileIo.h"
#include "ImageIo.h"
#include "ProgramOptions.h"

using namespace std;
using namespace std::chrono;

GB::Keys g_keys = {};

constexpr int JOYSTICK_DEAD_ZONE = 12000;

enum class Action {
    None,
    Quit,
    KeyUpdate,
    JoyUpdate,
};

Action PollEvents()
{
    Action eAction = Action::None;

    SDL_Event event;
    SDL_PollEvent(&event);

    const auto prev_keys = g_keys;
    switch (event.type) {
    case SDL_QUIT: eAction = Action::Quit; break;
    case SDL_KEYDOWN:
    case SDL_KEYUP: {
        const unsigned int value = (event.type == SDL_KEYDOWN) ? 1 : 0;
        switch (event.key.keysym.sym) {
        case SDLK_LEFT: g_keys.detail.dpad.detail.left = value; break;
        case SDLK_RIGHT: g_keys.detail.dpad.detail.right = value; break;
        case SDLK_UP: g_keys.detail.dpad.detail.up = value; break;
        case SDLK_DOWN: g_keys.detail.dpad.detail.down = value; break;
        case SDLK_z: g_keys.detail.buttons.detail.b = value; break;
        case SDLK_x: g_keys.detail.buttons.detail.a = value; break;
        case SDLK_c: g_keys.detail.buttons.detail.select = value; break;
        case SDLK_v: g_keys.detail.buttons.detail.start = value; break;
        };
        eAction = Action::KeyUpdate;
        break;
    }
    case SDL_CONTROLLERBUTTONUP:
    case SDL_CONTROLLERBUTTONDOWN:
    case SDL_CONTROLLERAXISMOTION: eAction = Action::JoyUpdate; break;
    }

    if (eAction == Action::KeyUpdate) {
        if (prev_keys.value == g_keys.value) {
            eAction = Action::None;
        }
    }

    return eAction;
}

struct AudioContext {
    mutex                  mtx;
    const GB::AudioSample* pBuf     = nullptr;
    size_t                 buf_size = 0;
    size_t                 pos      = 0;
    size_t                 prev_pos = 0;
};

enum ReturnCode : int {
    ReturnCode_OK                           = 0,
    ReturnCode_INVALID_ROM_FILE             = 1,
    ReturnCode_INVALID_SAVE_FILE            = 2,
    ReturnCode_INVALID_BOOT_ROM             = 3,
    ReturnCode_SDL_WINDOW_ERROR             = 4,
    ReturnCode_SDL_RENDERER_ERROR           = 5,
    ReturnCode_SDL_TEXTURE_ERROR            = 6,
    ReturnCode_SDL_AUDIO_ERROR              = 7,
    ReturnCode_COMMAND_LINE_PARAMETER_ERROR = 8,
};

void SDLAudioCallback(void* userdata, Uint8* stream, int len)
{
    if (len <= 0) {
        return;
    }

    auto pContext = reinterpret_cast<AudioContext*>(userdata);

    size_t pos = 0;
    {
        lock_guard<mutex> lock(pContext->mtx);
        pos = pContext->pos;
    }

    const size_t nb_samples_needed = (size_t)len / sizeof(GB::AudioSample);
    const size_t nb_samples_have   = pos - pContext->prev_pos;

    if (nb_samples_have < nb_samples_needed) {
        // Fill with silence
        memset(stream, 0, (size_t)len);
        return;
    }

    const size_t read_pos     = pContext->prev_pos + nb_samples_needed;
    const size_t buf_pos      = read_pos % pContext->buf_size;
    const size_t buf_prev_pos = pContext->prev_pos % pContext->buf_size;

    if (buf_pos > buf_prev_pos) {
        // No wrap around
        memcpy(stream, reinterpret_cast<const Uint8*>(&pContext->pBuf[buf_prev_pos]), (size_t)len);
    } else {
        const size_t till_end       = pContext->buf_size - buf_prev_pos;
        const size_t till_end_bytes = till_end * sizeof(GB::AudioSample);
        memcpy(stream, reinterpret_cast<const Uint8*>(&pContext->pBuf[buf_prev_pos]), till_end_bytes);
        memcpy(stream + till_end_bytes,
               reinterpret_cast<const Uint8*>(&pContext->pBuf),
               (size_t)len - till_end_bytes);
    }

    pContext->prev_pos = read_pos;
}

int main(int argc, char* argv[])
{
    auto upProg_opts = ParseCommandLine(argc, argv);

    if (!upProg_opts) {
        return ReturnCode_COMMAND_LINE_PARAMETER_ERROR;
    }

    const auto& rProg_opts = *upProg_opts; 

    const bool use_SDL   = rProg_opts.use_SDL;
    const bool use_vsync = rProg_opts.use_vsync;
    const bool save_screenshot_on_exit = rProg_opts.save_screenshot_on_exit;

    const uint64_t run_frame_count = rProg_opts.run_frame_count; // 0 = unlimited

    const double speed_factor = rProg_opts.speed_factor; // 0 = unlimited

    // const char* boot_rom_file_name = R"(D:\emu\gb\DMG_ROM.bin)";
    const string boot_rom_file_name = rProg_opts.boot_ROM_file;
    const char*  file_name          = rProg_opts.ROM_file.c_str();

    // clang-format off
    // file_name = R"(D:\emu\gb\PD\Dan Laser Demo (PD).gb)";
    // file_name = R"(D:\emu\gb\PD\MegAnime by Megaman_X (PD).gb)";
    // file_name = R"(D:\emu\gb\PD\Apocalypse Now Demo (PD).gb)";
    // file_name = R"(D:\emu\gb\PD\Kirby XXL Demo (PD).gb)";
    // file_name = R"(D:\emu\gb\PD\pocket.gb)";
    // file_name = R"(D:\emu\gb\PD\gejmboj.gb)";
    // file_name = R"(D:\emu\gb\PD\20y.gb)";
    // file_name = R"(D:\emu\gb\PD\jml-sam.gb)";
    // file_name = R"(D:\emu\gb\PD\jmla09.gb)";
    // file_name = R"(D:\emu\gb\PD\oh.gb)";
    // file_name = R"(D:\emu\gb\PD\Dawn.gb)";

    // file_name = R"(D:\emu\gb\Asteroids (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Alleyway (World).gb)";
    // file_name = R"(D:\emu\gb\Amazing Penguin (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Altered Space - A 3-D Alien Adventure (USA).gb)";
    // file_name = R"(D:\emu\gb\Battle Arena Toshinden (USA) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Bomberman GB (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Chikyuu Kaihou Gun ZAS (Japan).gb)";
    // file_name = R"(D:\emu\gb\Castlevania - The Adventure (USA).gb)";
    // file_name = R"(D:\emu\gb\Castlevania II - Belmont's Revenge (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Castlevania Legends (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Donkey Kong (World) (Rev A) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Donkey Kong Land (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Earthworm Jim (USA).gb)";
    // file_name = R"(D:\emu\gb\Kirby's Dream Land (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Foreman for Real (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Gauntlet II (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Gremlins 2 - The New Batch (World).gb)";
    // file_name = R"(D:\emu\gb\Ken Griffey Jr. Presents Major League Baseball (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Killer Instinct (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Krusty's Fun House (USA, Europe).gb)";  file_name = R"(D:\emu\gb\Lion King, The (USA).gb)";
    // file_name = R"(D:\emu\gb\Lethal Weapon (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Legend of Zelda, The - Link's Awakening (Canada).gb)";
    // file_name = R"(D:\emu\gb\Mario's Picross (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Metroid II - Return of Samus (World).gb)";
    // file_name = R"(D:\emu\gb\Mega Man - Dr. Wily's Revenge (USA).gb)";
    // file_name = R"(D:\emu\gb\Mega Man II (USA).gb)";
    // file_name = R"(D:\emu\gb\Mega Man III (USA).gb)";
    // file_name = R"(D:\emu\gb\Mega Man IV (USA).gb)";
    // file_name = R"(D:\emu\gb\Mega Man V (USA) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Mortal Kombat II (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Mortal Kombat 3 (USA).gb)";
    // file_name = R"(D:\emu\gb\Mortal Kombat 4 (USA, Europe) (SGB Enhanced).gbc)";
    // file_name = R"(D:\emu\gb\Monster Rancher Battle Card GB (USA) (SGB Enhanced).gbc)";
    // file_name = R"(D:\emu\gb\Mighty Morphin Power Rangers (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Nemesis (USA).gb)";
    // file_name = R"(D:\emu\gb\Panel Action Bingo (USA).gb)";
    // file_name = R"(D:\emu\gb\Shanghai (USA).gb)";
    // file_name = R"(D:\emu\gb\Sports Illustrated - Football & Baseball (USA).gb)";
    // file_name = R"(D:\emu\gb\Spot - The Cool Adventure (USA).gb)";
    // file_name = R"(D:\emu\gb\Super Mario Land (World) (Rev A).gb)";
    // file_name = R"(D:\emu\gb\Super Mario Land 2 - 6 Golden Coins (USA, Europe) (Rev B).gb)";
    // file_name = R"(D:\emu\gb\Super R.C. Pro-Am (USA, Europe).gb)";
    // file_name = R"(D:\emu\gb\Trip World (Europe).gb)";
    // file_name = R"(D:\emu\gb\V-Rally - Championship Edition (Europe) (En,Fr,De).gb)";
    // file_name = R"(D:\emu\gb\Wario Blast featuring Bomberman! (USA, Europe) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Wario Land - Super Mario Land 3 (World).gb)";
    // file_name = R"(D:\emu\gb\Game & Watch Gallery 3 (USA, Europe) (SGB Enhanced).gbc)";
    // file_name = R"(D:\emu\gb\NBA 3 on 3 featuring Kobe Bryant (USA) (SGB Enhanced).gbc)";
    // file_name = R"(D:\emu\gb\Godzilla - The Series (USA) (En,Fr,De).gbc)";
    // file_name = R"(D:\emu\gb\Smurfs, The (Europe) (En,Fr,De,Es).gb)";
    // file_name = R"(D:\emu\gb\Dr. Franken (Europe) (En,Fr,De,Es,It,Nl,Sv).gb)";
    // file_name = R"(D:\emu\gb\Aretha II (Japan).gb)";
    // file_name = R"(D:\emu\gb\Zerd no Densetsu (Japan).gb)";
    // file_name = R"(D:\emu\gb\Magic Knight RayEarth 2nd. - The Missing Colors (Japan) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Dai-2-ji Super Robot Taisen G (Japan) (SGB Enhanced).gb)";
    // file_name = R"(D:\emu\gb\Final Fantasy Legend, The (USA).gb)";


    // file_name = R"(D:\emu\gb\Tests\cpu_instrs\cpu_instrs.gb)";
    // file_name = R"(D:\emu\gb\Tests\cpu_instrs\individual\02-interrupts.gb)";
    // file_name = R"(D:\emu\gb\Tests\instr_timing\instr_timing.gb)";

    // file_name = R"(D:\emu\gb\Tests\dmg_sound\dmg_sound.gb)";

    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\01-registers.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\02-len ctr.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\03-trigger.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\04-sweep.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\05-sweep details.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\06-overflow on trigger.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\07-len sweep period sync.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\08-len ctr during power.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\09-wave read while on.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\10-wave trigger while on.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\11-regs after power.gb)";
    // file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\12-wave write while on.gb)";

    // file_name = R"(D:\emu\gb\Tests\mem_timing\mem_timing.gb)";
    // file_name = R"(D:\emu\gb\Tests\mem_timing-2\mem_timing.gb)";
    // file_name = R"(D:\emu\gb\Tests\interrupt_time\interrupt_time.gb)";
    // file_name = R"(D:\emu\gb\Tests\halt_bug.gb)";

    // file_name = R"()";
    // clang-format on

    const vector<char> buffer = LoadFile(file_name);

    if (buffer.empty()) {
        return ReturnCode_INVALID_ROM_FILE;
    }

    GB::GB gb;

    g_keys.value = 0xFF;

    if (!gb.Load(buffer.data(), buffer.size())) {
        return ReturnCode_INVALID_ROM_FILE;
    }

    const bool use_boot_rom = !boot_rom_file_name.empty();
    if (use_boot_rom) {
        const vector<char> boot_rom = LoadFile(boot_rom_file_name.c_str());

        if (boot_rom.size() != 256) {
            return ReturnCode_INVALID_BOOT_ROM;
        }

        gb.LoadBootROM(boot_rom.data(), boot_rom.size());
    }

    const GB::ROM_Header& rHeader = gb.RefHeader();

    gb.Reset(use_boot_rom ? GB::ResetOption_USE_BOOT_ROM : GB::ResetOption_NONE);

    string save_filename;
    if (rHeader.RAM_size > 0) {
        // Load save RAM file
        save_filename = GetSaveFileName(file_name);

        if (!save_filename.empty()) {
            printf("Loading save file %s...\n", save_filename.c_str());

            vector<char> save_buf = LoadSaveFile(save_filename.c_str(), rHeader.RAM_size);

            if (!save_buf.empty()) {
                if (!gb.LoadSaveRAM(save_buf.data(), save_buf.size())) {
                    printf("ERROR:Failed to load save file\n");
                    return ReturnCode_INVALID_SAVE_FILE;
                }
            }
        }
    }

    const uint32_t zoom = rProg_opts.video_scale;

    AudioContext audio_context = {};

    audio_context.buf_size = gb.GetAudioBufSize();
    audio_context.pBuf     = gb.GetAudioBuf();

    SDL_Window*         pWindow    = nullptr;
    SDL_Renderer*       pRenderer  = nullptr;
    SDL_Texture*        pTexture   = nullptr;
    SDL_AudioDeviceID   audio_dev  = 0;
    SDL_GameController* controller = nullptr;

    if (use_SDL) {
        SDL_Init(SDL_INIT_EVERYTHING);

        pWindow = SDL_CreateWindow("Ataboy",
                                   SDL_WINDOWPOS_UNDEFINED,
                                   SDL_WINDOWPOS_UNDEFINED,
                                   160 * zoom,
                                   144 * zoom,
                                   SDL_WINDOW_SHOWN);

        if (!pWindow) {
            printf("Can't initialize SDL window\n");
            return ReturnCode_SDL_WINDOW_ERROR;
        }

        Uint32 renderer_flags = 0;

        if (use_vsync) {
            renderer_flags |= SDL_RENDERER_PRESENTVSYNC;
        }

        pRenderer = SDL_CreateRenderer(pWindow, -1, renderer_flags);

        if (!pRenderer) {
            printf("Can't initialize SDL renderer\n");
            return ReturnCode_SDL_RENDERER_ERROR;
        }

        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
        SDL_RenderSetLogicalSize(pRenderer, 160, 144);
        pTexture =
            SDL_CreateTexture(pRenderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, 160, 144);

        if (!pTexture) {
            printf("Can't initialize SDL texture\n");
            return ReturnCode_SDL_TEXTURE_ERROR;
        }

        SDL_AudioSpec audio_want = {};
        SDL_AudioSpec audio_have = {};

        audio_want.callback = &SDLAudioCallback;
        audio_want.channels = 2;
        audio_want.format   = AUDIO_S16LSB;
        audio_want.freq     = 32768;
        audio_want.samples  = 512;
        audio_want.userdata = &audio_context;

        audio_dev = SDL_OpenAudioDevice(nullptr, 0, &audio_want, &audio_have, 0);

        if (!audio_dev) {
            printf("Can't initialize SDL audio\n");
            return ReturnCode_SDL_AUDIO_ERROR;
        }

        for (int i = 0; i < SDL_NumJoysticks(); ++i) {
            if (SDL_IsGameController(i)) {
                controller = SDL_GameControllerOpen(i);
                if (controller) {
                    break;
                } else {
                    printf("Could not open gamecontroller %i: %s\n", i, SDL_GetError());
                }
            }
        }
    }

    auto start_time = high_resolution_clock::now();
    int  nb_frames  = 0;

    if (use_SDL) {
        SDL_PauseAudioDevice(audio_dev, 0);
    }

    GB::u32 last_rendered_frame = 0;
    auto    run_time            = high_resolution_clock::now();
    auto    pause_time          = nanoseconds(0);

    bool bQuit = false;
    while (!bQuit) {
        if (use_SDL) {
            Action eAction = Action::None;
            do {
                eAction = PollEvents();

                if (eAction == Action::JoyUpdate && controller) {
                    auto prev_keys = g_keys;

                    auto fnCheckButton = [&](auto eButton, auto fnOut) {
                        if (SDL_GameControllerGetButton(controller, eButton)) {                                                        
                            fnOut(1);                                                                                               
                        } else {                                                                                                 
                            fnOut(0);                                                                                               
                        }
                    };

#define SET_BIT(x) [](auto val) {x = val;}

                    fnCheckButton(SDL_CONTROLLER_BUTTON_DPAD_LEFT, SET_BIT(g_keys.detail.dpad.detail.left));
                    fnCheckButton(SDL_CONTROLLER_BUTTON_DPAD_RIGHT, SET_BIT(g_keys.detail.dpad.detail.right));
                    fnCheckButton(SDL_CONTROLLER_BUTTON_DPAD_UP, SET_BIT(g_keys.detail.dpad.detail.up));
                    fnCheckButton(SDL_CONTROLLER_BUTTON_DPAD_DOWN, SET_BIT(g_keys.detail.dpad.detail.down));
                    fnCheckButton(SDL_CONTROLLER_BUTTON_A, SET_BIT(g_keys.detail.buttons.detail.a));
                    fnCheckButton(SDL_CONTROLLER_BUTTON_X, SET_BIT(g_keys.detail.buttons.detail.b));
                    fnCheckButton(SDL_CONTROLLER_BUTTON_START, SET_BIT(g_keys.detail.buttons.detail.start));
                    fnCheckButton(SDL_CONTROLLER_BUTTON_BACK, SET_BIT(g_keys.detail.buttons.detail.select));

#undef SET_BIT

                    const auto x_axis = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
                    if (x_axis < -JOYSTICK_DEAD_ZONE) {
                        g_keys.detail.dpad.detail.left = 1;
                    } else if (x_axis > JOYSTICK_DEAD_ZONE) {
                        g_keys.detail.dpad.detail.right = 1;
                    }

                    const auto y_axis = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
                    if (y_axis < -JOYSTICK_DEAD_ZONE) {
                        g_keys.detail.dpad.detail.up = 1;
                    } else if (y_axis > JOYSTICK_DEAD_ZONE) {
                        g_keys.detail.dpad.detail.down = 1;
                    }

                    if (prev_keys.value != g_keys.value) {
                        eAction = Action::KeyUpdate;
                    }
                }

                if (eAction == Action::Quit) {
                    bQuit = true;
                    break;
                } else if (eAction == Action::KeyUpdate) {
                    gb.UpdateKeys(g_keys);
                }

            } while (eAction != Action::None);

            {
                lock_guard<mutex> lock(audio_context.mtx);
                audio_context.pos = gb.GetAudioBufPos();
            }
        }

        if (bQuit) {
            break;
        }

        const auto cur_frame_number = gb.GetFrameNumber();

        const bool frame_count_exceeded = run_frame_count > 0 && (int)cur_frame_number >= run_frame_count;

        if (cur_frame_number != last_rendered_frame) {
            ++nb_frames;
            last_rendered_frame = cur_frame_number;

            const bool save_screenshot = save_screenshot_on_exit && frame_count_exceeded;

            if (use_SDL || save_screenshot) {
                const auto pPix = gb.GetPixels();

                if (use_SDL) {
                    SDL_UpdateTexture(pTexture, nullptr, pPix, sizeof(pPix[0]) * 160);

                    SDL_RenderClear(pRenderer);
                    SDL_RenderCopy(pRenderer, pTexture, NULL, NULL);
                    SDL_RenderPresent(pRenderer);
                }

                if (save_screenshot) {
                    string image_name = GetScreenShotFileName(file_name);

                    if (!image_name.empty()) {
                        SaveImage(image_name.c_str(), pPix);
                    }
                }
            }
        }

        if (speed_factor == 0) {
            gb.RunTime(nanoseconds(16666667));
        } else {
            const auto time_now = high_resolution_clock::now();

            auto elapsed = time_now - run_time - pause_time;

            constexpr auto MAX_SKIP_TIME = milliseconds(50);
            if (elapsed > MAX_SKIP_TIME) {
                pause_time += elapsed - MAX_SKIP_TIME;
                elapsed = MAX_SKIP_TIME;
            }

            const nanoseconds elapsed_ns = duration_cast<nanoseconds>(elapsed);

            const auto elapsed_gb_time =
                nanoseconds(static_cast<uint64_t>(elapsed_ns.count() * speed_factor));

            const auto emu_elapsed = gb.RunTime(elapsed_gb_time);

            const auto emu_alapsed_clock_time =
                nanoseconds(static_cast<uint64_t>(emu_elapsed.count() / speed_factor));

            run_time += emu_alapsed_clock_time;

            if (start_time + seconds(1) < time_now) {
                const duration<float> fsec = time_now - start_time;
                const float           fps  = 1.0f * nb_frames / fsec.count();
                printf("FPS:%.2f (%.0f%%)\r", fps, fps / 60 * 100);
                nb_frames  = 0;
                start_time = time_now;
            }

            this_thread::sleep_for(milliseconds(1));
        }

        if (frame_count_exceeded) {
            break;
        }
    }

    if (!save_filename.empty()) {
        const auto save_RAM_info = gb.RefSaveRAM();
        SaveSaveFile(save_filename.c_str(), save_RAM_info.first, save_RAM_info.second);
    }

    if (use_SDL) {
        SDL_PauseAudioDevice(audio_dev, 1);
        SDL_AudioQuit();
        SDL_Quit();
    }

    return ReturnCode_OK;
}

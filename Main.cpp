// This is an open source non-commercial project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

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

#include <vector>
#include <fstream>
#include <cstdio>
#include <array>
#include <thread>
#include <chrono>
#include <mutex>


#include <SDL2/SDL.h>

#include "FileIo.h"

using namespace std;
using namespace std::chrono;

GB::Keys g_keys = {};

constexpr int JOYSTICK_DEAD_ZONE = 12000;

enum class Action
{
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
    switch(event.type) {
    case SDL_QUIT: 
        eAction = Action::Quit;
        break;
    case SDL_KEYDOWN:
        switch(event.key.keysym.sym){
        case SDLK_LEFT:   g_keys.k.left = 0;  break;
        case SDLK_RIGHT:  g_keys.k.right = 0; break;
        case SDLK_UP:     g_keys.k.up = 0;    break;
        case SDLK_DOWN:   g_keys.k.down = 0;  break;
        case SDLK_z:      g_keys.k.b = 0;     break;
        case SDLK_x:      g_keys.k.a = 0;     break;
        case SDLK_c:      g_keys.k.select = 0;break;
        case SDLK_v:      g_keys.k.start = 0; break;
        }; 
        eAction = Action::KeyUpdate;
        break;
    case SDL_KEYUP :
        switch(event.key.keysym.sym){
        case SDLK_LEFT:   g_keys.k.left = 1;  break;
        case SDLK_RIGHT:  g_keys.k.right = 1; break;
        case SDLK_UP:     g_keys.k.up = 1;    break;
        case SDLK_DOWN:   g_keys.k.down = 1;  break;
        case SDLK_z:      g_keys.k.b = 1;     break;
        case SDLK_x:      g_keys.k.a = 1;     break;
        case SDLK_c:      g_keys.k.select = 1;break;
        case SDLK_v:      g_keys.k.start = 1; break;
        }
        eAction = Action::KeyUpdate;
        break;
    case SDL_CONTROLLERBUTTONUP:
    case SDL_CONTROLLERBUTTONDOWN:
    case SDL_CONTROLLERAXISMOTION:
        eAction = Action::JoyUpdate;
        break;
    }

    if (eAction == Action::KeyUpdate)
    {
        if (prev_keys.value == g_keys.value) {
            eAction = Action::None;
        }
    }

    return eAction;
}

struct AudioContext
{
    mutex mtx;
    const GB::AudioSample* pBuf = nullptr;
    size_t buf_size = 0;
    size_t pos = 0;
    size_t prev_pos = 0;
};

void SDLAudioCallback(void* userdata, Uint8* stream, int len)
{
    auto pContext = reinterpret_cast<AudioContext*>(userdata);

    size_t pos = 0;
    {
        lock_guard<mutex> lock(pContext->mtx);
        pos = pContext->pos;
    }

    const size_t nb_samples_needed = len / sizeof(GB::AudioSample);
    const size_t nb_samples_have = pos - pContext->prev_pos;

    if (nb_samples_have < nb_samples_needed)
    {
        // Fill with silence
        memset(stream, 0, len);
        return;
    }

    const size_t read_pos = pContext->prev_pos + nb_samples_needed;
    const size_t buf_pos = read_pos % pContext->buf_size;
    const size_t buf_prev_pos = pContext->prev_pos % pContext->buf_size;

    if (buf_pos > buf_prev_pos)
    {
        // No wrap around
        memcpy(stream, reinterpret_cast<const Uint8*>(&pContext->pBuf[buf_prev_pos]), len);
    }
    else
    {
        const size_t till_end = pContext->buf_size - buf_prev_pos;
        const size_t till_end_bytes = till_end * sizeof(GB::AudioSample);
        memcpy(stream, reinterpret_cast<const Uint8*>(&pContext->pBuf[buf_prev_pos]), till_end_bytes);
        memcpy(stream + till_end_bytes, reinterpret_cast<const Uint8*>(&pContext->pBuf), (size_t)len - till_end_bytes);
    }

    pContext->prev_pos = read_pos;
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    bool use_SDL = true;
    bool use_vsync = true;
    int speed_factor = 1;

    const char* file_name = nullptr;
    if (argc > 1)
    {
        file_name = argv[1];
    }
    //file_name = R"(D:\emu\gb\PD\Dan Laser Demo (PD).gb)";
    //file_name = R"(D:\emu\gb\PD\MegAnime by Megaman_X (PD).gb)";
    //file_name = R"(D:\emu\gb\PD\Apocalypse Now Demo (PD).gb)";
    //file_name = R"(D:\emu\gb\PD\Kirby XXL Demo (PD).gb)";
    //file_name = R"(D:\emu\gb\PD\pocket.gb)";
    //file_name = R"(D:\emu\gb\PD\gejmboj.gb)";
    //file_name = R"(D:\emu\gb\PD\20y.gb)";
    //file_name = R"(D:\emu\gb\PD\jml-sam.gb)";
    //file_name = R"(D:\emu\gb\PD\jmla09.gb)";
    //file_name = R"(D:\emu\gb\PD\oh.gb)";
    //file_name = R"(D:\emu\gb\PD\Dawn.gb)";


    //file_name = R"(D:\emu\gb\Asteroids (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Alleyway (World).gb)";
    //file_name = R"(D:\emu\gb\Amazing Penguin (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Altered Space - A 3-D Alien Adventure (USA).gb)";
    //file_name = R"(D:\emu\gb\Battle Arena Toshinden (USA) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Bomberman GB (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Chikyuu Kaihou Gun ZAS (Japan).gb)";
    //file_name = R"(D:\emu\gb\Castlevania - The Adventure (USA).gb)";
    //file_name = R"(D:\emu\gb\Castlevania II - Belmont's Revenge (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Castlevania Legends (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Donkey Kong (World) (Rev A) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Donkey Kong Land (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Earthworm Jim (USA).gb)";
    //file_name = R"(D:\emu\gb\Kirby's Dream Land (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Foreman for Real (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Gauntlet II (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Gremlins 2 - The New Batch (World).gb)";
    //file_name = R"(D:\emu\gb\Ken Griffey Jr. Presents Major League Baseball (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Killer Instinct (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Krusty's Fun House (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Lion King, The (USA).gb)";
    //file_name = R"(D:\emu\gb\Lethal Weapon (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Legend of Zelda, The - Link's Awakening (Canada).gb)";
    //file_name = R"(D:\emu\gb\Mario's Picross (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Metroid II - Return of Samus (World).gb)";
    //file_name = R"(D:\emu\gb\Mega Man - Dr. Wily's Revenge (USA).gb)";
    //file_name = R"(D:\emu\gb\Mega Man II (USA).gb)";
    //file_name = R"(D:\emu\gb\Mega Man III (USA).gb)";
    //file_name = R"(D:\emu\gb\Mega Man IV (USA).gb)";
    //file_name = R"(D:\emu\gb\Mega Man V (USA) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Mortal Kombat II (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Mortal Kombat 3 (USA).gb)";
    //file_name = R"(D:\emu\gb\Mortal Kombat 4 (USA, Europe) (SGB Enhanced).gbc)";
    //file_name = R"(D:\emu\gb\Monster Rancher Battle Card GB (USA) (SGB Enhanced).gbc)";
    //file_name = R"(D:\emu\gb\Mighty Morphin Power Rangers (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Nemesis (USA).gb)";
    //file_name = R"(D:\emu\gb\Panel Action Bingo (USA).gb)";
    //file_name = R"(D:\emu\gb\Shanghai (USA).gb)";
    //file_name = R"(D:\emu\gb\Sports Illustrated - Football & Baseball (USA).gb)";
    //file_name = R"(D:\emu\gb\Spot - The Cool Adventure (USA).gb)";
    //file_name = R"(D:\emu\gb\Super Mario Land (World) (Rev A).gb)";
    //file_name = R"(D:\emu\gb\Super Mario Land 2 - 6 Golden Coins (USA, Europe) (Rev B).gb)";
    //file_name = R"(D:\emu\gb\Super R.C. Pro-Am (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Trip World (Europe).gb)";
    //file_name = R"(D:\emu\gb\V-Rally - Championship Edition (Europe) (En,Fr,De).gb)";
    //file_name = R"(D:\emu\gb\Wario Blast featuring Bomberman! (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Wario Land - Super Mario Land 3 (World).gb)";
    //file_name = R"(D:\emu\gb\Game & Watch Gallery 3 (USA, Europe) (SGB Enhanced).gbc)";
    //file_name = R"(D:\emu\gb\NBA 3 on 3 featuring Kobe Bryant (USA) (SGB Enhanced).gbc)";
    //file_name = R"(D:\emu\gb\Godzilla - The Series (USA) (En,Fr,De).gbc)";




    //file_name = R"(D:\emu\gb\Tests\cpu_instrs\cpu_instrs.gb)";
    //file_name = R"(D:\emu\gb\Tests\cpu_instrs\individual\02-interrupts.gb)";
    //file_name = R"(D:\emu\gb\Tests\instr_timing\instr_timing.gb)";

    //file_name = R"(D:\emu\gb\Tests\dmg_sound\dmg_sound.gb)";
    
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\01-registers.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\02-len ctr.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\03-trigger.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\04-sweep.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\05-sweep details.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\06-overflow on trigger.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\07-len sweep period sync.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\08-len ctr during power.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\09-wave read while on.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\10-wave trigger while on.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\11-regs after power.gb)";
    //file_name = R"(D:\emu\gb\Tests\dmg_sound\rom_singles\12-wave write while on.gb)";

    //file_name = R"(D:\emu\gb\Tests\mem_timing\mem_timing.gb)";
    //file_name = R"(D:\emu\gb\Tests\mem_timing-2\mem_timing.gb)";
    //file_name = R"(D:\emu\gb\Tests\interrupt_time\interrupt_time.gb)";

    //file_name = R"()";

    const auto buffer = LoadFile(file_name);

    if (buffer.empty())
    {
        return 1;
    }

    GB::GB gb;

    g_keys.value = 0xFF;

    if (!gb.Load(buffer.data(), buffer.size()))
    {
        return 1;
    }

    gb.Reset();

    const int zoom = 4;

    AudioContext audio_context = {};

    audio_context.buf_size = gb.GetAudioBufSize();
    audio_context.pBuf = gb.GetAudioBuf();

    SDL_Window* pWindow = nullptr;
    SDL_Renderer* pRenderer = nullptr;
    SDL_Texture* pTexture = nullptr;
    SDL_AudioDeviceID audio_dev = 0;
    SDL_GameController* controller = nullptr;

    if (use_SDL)
    {
        SDL_Init(SDL_INIT_EVERYTHING);

        pWindow = SDL_CreateWindow("Ataboy", 
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
            160 * zoom, 144 * zoom, SDL_WINDOW_SHOWN);

        if (!pWindow)
        {
            printf("Can't initialize SDL window\n");
            return 1;
        }

        Uint32 renderer_flags = 0;

        if (use_vsync)
        {
            renderer_flags |= SDL_RENDERER_PRESENTVSYNC;
        }

        pRenderer = SDL_CreateRenderer(pWindow, -1, renderer_flags);

        if (!pRenderer)
        {
            printf("Can't initialize SDL renderer\n");
            return 1;
        }

        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
        SDL_RenderSetLogicalSize(pRenderer, 160, 144);
        pTexture = SDL_CreateTexture(pRenderer,
            SDL_PIXELFORMAT_ARGB8888,
            SDL_TEXTUREACCESS_STREAMING,
            160, 144);

        if (!pTexture)
        {
            printf("Can't initialize SDL texture\n");
            return 1;
        }

        SDL_AudioSpec audio_want = {};
        SDL_AudioSpec audio_have = {};

        audio_want.callback = &SDLAudioCallback;
        audio_want.channels = 2;
        audio_want.format = AUDIO_S16LSB;
        audio_want.freq = 32768;
        audio_want.samples = 512;
        audio_want.userdata = &audio_context;

        audio_dev = SDL_OpenAudioDevice(nullptr, 0, &audio_want, &audio_have, 0);
    
        if (!audio_dev)
        {
            printf("Can't initialize SDL audio\n");
            return 1;
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
    int nb_frames = 0;

    if (use_SDL)
    {
        SDL_PauseAudioDevice(audio_dev, 0);
    }

    GB::u32 last_rendered_frame = 0;
    auto run_time = high_resolution_clock::now();
    auto pause_time = nanoseconds(0);

    bool bQuit = false;
    while(!bQuit)
    {
        
        if (use_SDL)
        {
            Action eAction = Action::None;
            do
            {
                eAction = PollEvents();

                if (eAction == Action::JoyUpdate && controller)
                {
                    auto prev_keys = g_keys;
#define CHECK_BUTTON(x, y) if (SDL_GameControllerGetButton(controller, x)) { y = 0; } else { y = 1;}
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_DPAD_LEFT, g_keys.k.left);
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_DPAD_RIGHT, g_keys.k.right);
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_DPAD_UP, g_keys.k.up);
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_DPAD_DOWN, g_keys.k.down);
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_A, g_keys.k.a);
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_X, g_keys.k.b);
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_START, g_keys.k.start);
                    CHECK_BUTTON(SDL_CONTROLLER_BUTTON_BACK, g_keys.k.select);

                    const auto x_axis = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
                    if (x_axis < -JOYSTICK_DEAD_ZONE) {
                        g_keys.k.left = 0;
                    } else if (x_axis > JOYSTICK_DEAD_ZONE) {
                        g_keys.k.right = 0;
                    }

                    const auto y_axis = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
                    if (y_axis < -JOYSTICK_DEAD_ZONE) {
                        g_keys.k.up = 0;
                    } else if (y_axis > JOYSTICK_DEAD_ZONE) {
                        g_keys.k.down = 0;
                    }

                    if (prev_keys.value != g_keys.value)
                    {
                        eAction = Action::KeyUpdate;
                    }
                }

                if (eAction == Action::Quit)
                {
                    bQuit = true;
                    break;
                }
                else if (eAction == Action::KeyUpdate)
                {
                    gb.UpdateKeys(g_keys);
#ifndef _DEBUG
                    printf(""); // Workaround weird SDL bug
#endif
                }

            } while (eAction != Action::None);

            {
                lock_guard<mutex> lock(audio_context.mtx);
                audio_context.pos = gb.GetAudioBufPos();
            }


        }

        if (bQuit)
        {
            break;
        }


        const auto cur_frame_number = gb.GetFrameNumber();

        if (cur_frame_number != last_rendered_frame) {
            ++nb_frames;
            last_rendered_frame = cur_frame_number;

            if (use_SDL) {
                const auto pPix = gb.GetPixels();

                SDL_UpdateTexture(pTexture, nullptr, pPix, sizeof(pPix[0]) * 160);

                SDL_RenderClear(pRenderer);
                SDL_RenderCopy(pRenderer, pTexture, NULL, NULL);
                SDL_RenderPresent(pRenderer);
            }
        }

        const auto time_now = high_resolution_clock::now();

        auto elapsed = time_now - run_time - pause_time;

        constexpr auto MAX_SKIP_TIME = milliseconds(50);
        if (elapsed > MAX_SKIP_TIME)
        {
            pause_time += elapsed - MAX_SKIP_TIME;
            elapsed = MAX_SKIP_TIME;
        }

        const auto emu_elapsed = gb.RunTime(duration_cast<nanoseconds>(elapsed) * speed_factor);

        run_time += emu_elapsed / speed_factor;

        if (start_time + seconds(1) < time_now)
        {
            const duration<float> fsec = time_now - start_time;
            const float fps = 1.0f * nb_frames / fsec.count();
            printf("FPS:%.2f (%.0f%%) %d %d %d %d\r", fps, fps / 60 * 100, g_keys.k.left, g_keys.k.right, g_keys.k.up, g_keys.k.down);
            nb_frames = 0;
            start_time = time_now;
        }

        this_thread::sleep_for(milliseconds(1));
    }

    if (use_SDL)
    {
        SDL_PauseAudioDevice(audio_dev, 1);
        SDL_AudioQuit();
        SDL_Quit();
    }

    return 0;
}

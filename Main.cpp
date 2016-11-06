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


#include <SDL.h>

using namespace std;
using namespace std::chrono;

GB::Keys g_keys = {};

enum class Action
{
    None,
    Quit,
    KeyUpdate,
};

Action PollEvents()
{
    SDL_Event event;
    SDL_PollEvent(&event);

    switch(event.type)
    {
    case SDL_QUIT: return Action::Quit;
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
        }; return Action::KeyUpdate;
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
        // case SDLK_q: if(iGame > 0) {iGame--; bRestart = true; CPU::bQuit = true;} break;
        //case SDLK_w: if(iGame < iGameMax) {iGame++; bRestart = true; CPU::bQuit = true;} break;
        } return Action::KeyUpdate;
    }
    return Action::None;
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
    auto context = reinterpret_cast<AudioContext*>(userdata);

    size_t pos = 0;
    {
        lock_guard<mutex> lock(context->mtx);
        pos = context->pos;
    }

    size_t nb_samples_needed = len / sizeof(GB::AudioSample);
    size_t nb_samples_have = pos - context->prev_pos;

    if (nb_samples_have < nb_samples_needed)
    {
        // Fill with silence
        memset(stream, 0, len);
        return;
    }

    size_t read_pos = context->prev_pos + nb_samples_needed;
    size_t buf_pos = read_pos % context->buf_size;
    size_t buf_prev_pos = context->prev_pos % context->buf_size;

    if (buf_pos > buf_prev_pos)
    {
        // No wrap around
        memcpy(stream, reinterpret_cast<const Uint8*>(&context->pBuf[buf_prev_pos]), len);
    }
    else
    {
        size_t till_end = context->buf_size - buf_prev_pos;
        size_t till_end_bytes = till_end * sizeof(GB::AudioSample);
        memcpy(stream, reinterpret_cast<const Uint8*>(&context->pBuf[buf_prev_pos]), till_end_bytes);
        memcpy(stream + till_end_bytes, reinterpret_cast<const Uint8*>(&context->pBuf), (size_t)len - till_end_bytes);
    }

    context->prev_pos = read_pos;
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    bool bUseSDL = true;
    bool bUseVSync = true;

    const char* file_name = nullptr;
    if (argc > 1)
    {
        file_name = argv[1];
    }
    //file_name = R"(D:\emu\gb\Mortal Kombat II (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Battle Arena Toshinden (USA) (SGB Enhanced).gb)";
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
    //file_name = R"(D:\emu\gb\Chikyuu Kaihou Gun ZAS (Japan).gb)";
    //file_name = R"(D:\emu\gb\Wario Blast featuring Bomberman! (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Kirby's Dream Land (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Asteroids (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Mortal Kombat 3 (USA).gb)";
    //file_name = R"(D:\emu\gb\Amazing Penguin (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Foreman for Real (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Gremlins 2 - The New Batch (World).gb)";
    //file_name = R"(D:\emu\gb\Ken Griffey Jr. Presents Major League Baseball (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Killer Instinct (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Krusty's Fun House (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Lion King, The (USA).gb)";
    //file_name = R"(D:\emu\gb\Mario's Picross (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Mighty Morphin Power Rangers (USA, Europe) (SGB Enhanced).gb)";
    //file_name = R"(D:\emu\gb\Nemesis (USA).gb)";
    //file_name = R"(D:\emu\gb\Panel Action Bingo (USA).gb)";
    //file_name = R"(D:\emu\gb\Shanghai (USA).gb)";
    //file_name = R"(D:\emu\gb\Sports Illustrated - Football & Baseball (USA).gb)";
    //file_name = R"(D:\emu\gb\Spot - The Cool Adventure (USA).gb)";
    //file_name = R"(D:\emu\gb\Super Mario Land (World) (Rev A).gb)";
    //file_name = R"(D:\emu\gb\Super Mario Land 2 - 6 Golden Coins (USA, Europe) (Rev B).gb)";
    //file_name = R"(D:\emu\gb\Super R.C. Pro-Am (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Mortal Kombat 4 (USA, Europe) (SGB Enhanced).gbc)";
    //file_name = R"(D:\emu\gb\Castlevania - The Adventure (USA).gb)";
    //file_name = R"(D:\emu\gb\Castlevania II - Belmont's Revenge (USA, Europe).gb)";
    //file_name = R"(D:\emu\gb\Trip World (Europe).gb)";
    //file_name = R"(D:\emu\gb\Alleyway (World).gb)";
    //file_name = R"(D:\emu\gb\V-Rally - Championship Edition (Europe) (En,Fr,De).gb)";
    //file_name = R"(D:\emu\gb\Metroid II - Return of Samus (World).gb)";
    //file_name = R"(D:\emu\gb\Wario Land - Super Mario Land 3 (World).gb)";
    ifstream file(file_name, ios::binary | ios::ate);
    streamsize size = file.tellg();

    if (size < 0)
    {
        printf("Bad file\n");
        return 1;
    }

    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size))
    {
        return 1;
    }

    GB::GB gb;

    g_keys.value = 0xFF;

    gb.Load(buffer.data(), buffer.size());

    gb.Reset();

    int zoom = 4;

    AudioContext audio_context = {};

    audio_context.buf_size = gb.GetAudioBufSize();
    audio_context.pBuf = gb.GetAudioBuf();

    SDL_Window* pWindow = nullptr;
    SDL_Renderer* pRenderer = nullptr;
    SDL_Texture* pTexture = nullptr;
    SDL_AudioDeviceID audio_dev = 0;

    if (bUseSDL)
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

        if (bUseVSync)
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
    }

    auto start_time = high_resolution_clock::now();
    int nb_frames = 0;

    if (bUseSDL)
    {
        SDL_PauseAudioDevice(audio_dev, 0);
    }

    bool bQuit = false;
    while(!bQuit)
    {
        
        if (bUseSDL)
        {
            Action eAction = Action::None;
            do
            {
                eAction = PollEvents();

                if (eAction == Action::Quit)
                {
                    bQuit = true;
                    break;
                }
                else if (eAction == Action::KeyUpdate)
                {
                    gb.UpdateKeys(g_keys);
                    printf(""); // Fix weird SDL bug
                }

            } while (eAction != Action::None);
        }

        if (bQuit)
        {
            break;
        }

        gb.RunFrame();

        if (bUseSDL)
        {
            {
                lock_guard<mutex> lock(audio_context.mtx);
                audio_context.pos = gb.GetAudioBufPos();
            }

            auto pix = gb.GetPixels();

            SDL_UpdateTexture(pTexture, nullptr, pix, sizeof(pix[0]) * 160);

            SDL_RenderClear(pRenderer);
            SDL_RenderCopy(pRenderer, pTexture, NULL, NULL);
            SDL_RenderPresent(pRenderer);
        }
        ++nb_frames;
        auto time_now = high_resolution_clock::now();
        if (start_time + seconds(1) < time_now)
        {
            duration<float> fsec = time_now - start_time;
            float fps = 1.0f * nb_frames / fsec.count();
            printf("FPS:%.2f (%.0f%%)\n", fps, fps / 60 * 100);
            nb_frames = 0;
            start_time = time_now;
        }
    }



    if (bUseSDL)
    {
        SDL_PauseAudioDevice(audio_dev, 1);
        SDL_AudioQuit();
        SDL_Quit();
    }

    return 0;
}

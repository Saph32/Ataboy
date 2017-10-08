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

#include "ProgramOptions.h"

#include "cxxopts/cxxopts.hpp"

#include <iostream>

using namespace std;

std::unique_ptr<ProgramOptions> ParseCommandLine(int argc, char** argv)
{
    string help;

    try {
        cxxopts::Options opts("Ataboy", "Game Boy emulator");

        // clang-format off
        opts.add_options()
            ("s,sav", "SAV file path", cxxopts::value<string>())
            ("boot", "Boot ROM file path", cxxopts::value<string>())
            ("no-save-sav", "Don't save SAV file")
            ("no-load-sav", "Don't load SAV file")
            ("f,frames", "Run for x frames (0 = unlimited)", cxxopts::value<uint64_t>()->default_value("0"))
            ("no-vsync", "Do not synchronize video with vertical sync.")
            ("no-sdl", "Do not use SDL.")
            ("save-screenshot-on-exit", "Save a screenshot before exiting program.")
            ("speed", "Speed factor (1 = real time, <1 = slower, >1 = faster, 0 = unlimited)", cxxopts::value<double>()->default_value("1"))
            ("video-scale", "Video scaling factor", cxxopts::value<uint32_t>()->default_value("4"))
            ;
        // clang-format on

        opts.parse_positional("rom");
        opts.positional_help("ROM-file");

        help = opts.help();

        // Add positional option later so that it doesn't show in help
        opts.add_options()("rom", "ROM file path", cxxopts::value<string>());

        opts.parse(argc, argv);

        auto upProg_opts = make_unique<ProgramOptions>();

        auto fnCheckStringOpt = [&opts](const char* opt, string& out) {
            if (opts.count(opt)) {
                out = opts[opt].as<string>();
            }
        };

        fnCheckStringOpt("rom", upProg_opts->ROM_file);
        fnCheckStringOpt("sav", upProg_opts->SAV_file);
        fnCheckStringOpt("boot", upProg_opts->boot_ROM_file);

        upProg_opts->run_frame_count = opts["frames"].as<uint64_t>();
        upProg_opts->speed_factor    = opts["speed"].as<double>();
        upProg_opts->video_scale     = opts["video-scale"].as<uint32_t>();

        if (upProg_opts->video_scale < 1 || upProg_opts->video_scale > 64) {
            throw(cxxopts::OptionException("video-scale out of range"));
        }

        auto fnCheckBoolOpt = [&opts](const char* opt, bool& out, bool value) {
            if (opts.count(opt)) {
                out = value;
            }
        };

        fnCheckBoolOpt("no-save-sav", upProg_opts->save_SAV_file, false);
        fnCheckBoolOpt("no-load-sav", upProg_opts->load_SAV_file, false);
        fnCheckBoolOpt("no-vsync", upProg_opts->use_vsync, false);
        fnCheckBoolOpt("no-sdl", upProg_opts->use_SDL, false);
        fnCheckBoolOpt("save-screenshot-on-exit", upProg_opts->save_screenshot_on_exit, true);

        return upProg_opts;
    } catch (const cxxopts::OptionException& e) {
        cout << "error parsing options: " << e.what() << endl;
        cout << help << endl;
        return nullptr;
    }
}
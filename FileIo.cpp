// This is an open source non-commercial project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

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

#include "FileIo.h"

#include <fstream>
#include <cstdio>

#include <filesystem>

using namespace std;

static const std::string separator = filesystem::path("/").make_preferred().string();

vector<char> LoadFile(const char* file_name)
{

    vector<char> buf;

    if (!filesystem::exists(file_name)) {
        printf("ERROR : File %s doesn't exists\n", file_name);
        return {};
    }

    ifstream file(file_name, ios::binary);

    const size_t size = filesystem::file_size(file_name);
    if (size == 0) {
        printf("ERROR : File %s is empty\n", file_name);
        return {};
    }

    buf.resize(size);

    if (!file.read(buf.data(), size)) {
        return {};
    }

    return buf;
}

std::string GetSaveFileName(const char* gamepak_file_name)
{

    filesystem::path gamepak(gamepak_file_name);

    // Build save file name
    // Check with capital .SAV extension
    filesystem::path savefile = gamepak.parent_path().string() + separator + gamepak.stem().string() + ".SAV";
    if (exists(savefile)) {
        return savefile.string();
    }

    // Return default with lowercase extension
    savefile = gamepak.parent_path().string() + separator + gamepak.stem().string() + ".sav";
    return savefile.string();
}

std::vector<char> LoadSaveFile(const char* file_name, const size_t expected_size)
{

    std::vector<char> buf;

    if (!filesystem::exists(file_name)) {
        printf("WARNING : SAV file doesn't exists. Will create a new one.\n");
        return buf;
    }

    const size_t actual_file_size = filesystem::file_size(file_name);
    if (actual_file_size < expected_size) {
        printf(
            "WARNING : SAV file size too small (expected:%d got:%d). File will be ignored and overwritten.\n",
            (int)expected_size,
            (int)actual_file_size);
        return buf;
    }

    ifstream file(file_name, ios::binary);

    buf.resize(actual_file_size);

    if (!file.read(buf.data(), actual_file_size)) {
        printf("WARNING : Error reading SAV file. File will be ignored and overwritten.\n");
        return {};
    }

    return buf;
}

bool SaveSaveFile(const char* file_name, const char* data, const size_t size)
{

    ofstream out(file_name, ios::binary);

    if (!out) {
        printf("ERROR : Cannot write save file %s\n", file_name);
        return false;
    }

    out.write(data, size);

    return true;
}

std::string GetScreenShotFileName(const char* gamepak_file_name)
{

    filesystem::path gamepak(gamepak_file_name);

    int index = 0;
    while (index < 1000) {
        string suffix;

        if (index > 0) {
            suffix = "_" + to_string(index);
        }

        // Build save file name
        filesystem::path image_file =
            gamepak.parent_path().string() + separator + gamepak.stem().string() + suffix + ".PNG";

        // Check that it doesn't exists
        if (!exists(image_file)) {
            // Check that the capital version doesn't exists either
            image_file =
                gamepak.parent_path().string() + separator + gamepak.stem().string() + suffix + ".png";

            if (!exists(image_file)) {
                return image_file.string();
            }
        }

        // Try next index
        ++index;
    }

    // Failed
    return {};
}
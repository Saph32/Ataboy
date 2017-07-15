#pragma once

#include <vector>

std::vector<char> LoadFile(const char* file_name);
std::string GetSaveFileName(const char* gamepak_file_name);
std::vector<char> LoadSaveFile(const char* file_name, const size_t expected_size);
bool SaveSaveFile(const char* file_name, const char* data, const size_t size);
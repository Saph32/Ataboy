#pragma once

#include <vector>
#include <string>

std::vector<char> LoadFile(const char* file_name);
std::string GetSaveFileName(const char* gamepak_file_name);
std::vector<char> LoadSaveFile(const char* file_name, const std::size_t expected_size);
bool SaveSaveFile(const char* file_name, const char* data, const std::size_t size);
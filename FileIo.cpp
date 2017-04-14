#include "FileIo.h"

#include <fstream>
#include <cstdio>

using namespace std;

vector<char> LoadFile(const char * file_name)
{
    vector<char> buf;

    ifstream file(file_name, ios::binary | ios::ate);
    streamsize size = file.tellg();

    if (size < 0)
    {
        printf("Bad file\n");
        return {};
    }

    buf.resize(size);

    file.seekg(0, std::ios::beg);

    if (!file.read(buf.data(), size))
    {
        return{};
    }

    return buf;
}

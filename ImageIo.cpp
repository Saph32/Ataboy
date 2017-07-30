#include "ImageIo.h"

#include <png.h>

#include <cstdio>
#include <vector>

#include "at_scope_exit.h"

static bool png_error_occured = false;

static void PngError(png_structp, png_const_charp error_msg)
{
    printf("ERROR : PNG error : %s\n", error_msg);
    png_error_occured = true;
}
static void PngWarning(png_structp, png_const_charp warning_msg)
{
    printf("WARNING : PNG warning : %s\n", warning_msg);
}

bool SaveImage(const char * file_name, const uint32_t * pPixels)
{
    png_error_occured = false;

    FILE* pFile = fopen(file_name, "wb");

    if (!pFile)
    {
        printf("ERROR : Cannot write to file %s\n", file_name);
        return false;
    }

    AT_SCOPE_EXIT(fclose(pFile););

    png_structp pPng = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, PngError, PngWarning);

    if (!pPng)
    {
        printf("ERROR : png_create_write_struct failed\n");
        return false;
    }

    png_infop pInfo = nullptr;

    AT_SCOPE_EXIT(png_destroy_write_struct(&pPng, &pInfo););

    pInfo = png_create_info_struct(pPng);

    if (!pInfo)
    {
        printf("ERROR : png_create_info_struct failed\n");
        return false;
    }

    std::vector<png_bytep> rows(144);

    const uint32_t* pRow = pPixels;
    for (auto& rpRow : rows)
    {
        rpRow = reinterpret_cast<png_byte*>(const_cast<uint32_t*>(pRow));
        pRow += 160;
    }

    png_init_io(pPng, pFile);

    png_set_IHDR(
        pPng,
        pInfo,
        160, 144,
        8,
        PNG_COLOR_TYPE_RGBA,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT);

    png_write_info(pPng, pInfo);
    png_write_image(pPng, rows.data());
    png_write_end(pPng, nullptr);

    return png_error_occured;
}

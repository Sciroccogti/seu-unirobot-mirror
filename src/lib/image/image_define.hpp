#pragma once

#include <stddef.h>
#include <memory>
#include <map>

namespace imageproc
{
    struct VideoBuffer
    {
        unsigned char *start;
        size_t offset;
        size_t length;
        size_t bytesused;
        int lagtimems;
    };

    struct VideoBufferInfo
    {
        int width;
        int height;
        int size;
        unsigned int format;
    };

    enum ColorType
    {
        COLOR_CLEAN = 0,
        COLOR_GREEN = 1,
        COLOR_WHITE = 2
    };

    static const std::map<std::string, ColorType> color_name_map = {
        {"clean", COLOR_CLEAN},
        {"green", COLOR_GREEN},
        {"white", COLOR_WHITE}
    };

    inline std::string get_name_by_color(ColorType c)
    {
        for (auto &nm : color_name_map)
        {
            if (nm.second == c)
            {
                return nm.first;
            }
        }
        return "clean";
    }

    inline ColorType get_color_by_name(std::string name)
    {
        for (auto &nm : color_name_map)
        {
            if (nm.first == name)
            {
                return nm.second;
            }
        }
        return COLOR_CLEAN;
    }

    struct MinMax
    {
        float minimum;
        float maximum;
    };
    struct Color
    {
        ColorType c;
        MinMax H;
        MinMax S;
        MinMax I;
    };
}

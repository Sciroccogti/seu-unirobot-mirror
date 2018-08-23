#ifndef SEU_UNIROBOT_IMAGE_DEFINE_HPP
#define SEU_UNIROBOT_IMAGE_DEFINE_HPP

#include <stddef.h>
#include <memory>

namespace image
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

}
#endif
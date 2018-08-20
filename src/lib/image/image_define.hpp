#ifndef SEU_UNIROBOT_IMAGE_DEFINE_HPP
#define SEU_UNIROBOT_IMAGE_DEFINE_HPP

#include <stddef.h>
#include <memory>
#include "logger.hpp"

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
    
    class VideoFrame
    {
    public:
        int id;   /*!< Image number, starting from 0. */
        int width;    /*!< Image width. */
        int height;   /*!< Image height. */
        int channals; /*!< Image channal number, YUV444 is 3 and Gray is 1. */
        int size; /*!< Number of bytes used in image. */
        unsigned char *start;/*!< Image data start pointer. */

        VideoFrame()
        {
            start = nullptr;
        }

        /**
        * @brief     Construct an image.
        * @param w   Image width.
        * @param h   Image height.
        * @param c   Image channal number.
        * */
        VideoFrame(int w, int h, int c = 1)
        {
            id = 0;
            width = w;
            height = h;
            channals = c;
            size = width*height*channals;
            start = new unsigned char[w*h*c];
        }

        /**
        * @brief Copy an image.
        * @param f   Template image reference.
        * */
        VideoFrame(VideoFrame &f)
        {
            id = f.id;
            width = f.width;
            height = f.height;
            channals = f.channals;
            size = f.size;
        }

        /**
        * @brief Destroy an image, release memory and pointer.
        * */
        ~VideoFrame()
        {
            if (start != nullptr)
            {
                delete []start;
                start = nullptr;
            }
        }
    };
    
    typedef std::shared_ptr<VideoFrame> VideoFrame_Ptr;
}
#endif
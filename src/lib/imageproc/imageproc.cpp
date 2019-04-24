#include "imageproc.hpp"
#include <cmath>

using namespace cv;
using namespace std;

namespace vision
{
    float* rgb2rgbpf(const Mat &rgb)
    {
        int w = rgb.size().width;
        int h = rgb.size().height;
        float  *rgbpf = new float[h*w*3];
        int psize=w*h;
        for(int i=0;i<h;i++)
        {
            for(int j=0;j<w;j++)
            {
                unsigned int r = rgb.at<Vec3b>(i, j)[0];
                unsigned int g = rgb.at<Vec3b>(i, j)[1];
                unsigned int b = rgb.at<Vec3b>(i, j)[2];
                rgbpf[i*w+j] = static_cast<float>(r/255.0);
                rgbpf[psize+i*w+j] = static_cast<float>(g/255.0);
                rgbpf[2*psize+i*w+j] = static_cast<float>(b/255.0);
            }
        }
        return rgbpf;
    }
}


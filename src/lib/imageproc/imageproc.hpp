#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "darknet/network.h"
#include "model.hpp"

namespace vision
{
    float* rgb2rgbpf(const cv::Mat &rgb);
}

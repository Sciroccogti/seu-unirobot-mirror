#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "darknet/network.h"
#include "model.hpp"

namespace imageproc
{
    std::vector<object_prob> ball_and_post_detection(network &net, float *rgbfp, bool from_gpu, object_prob ball,
        object_prob post, int w, int h, float thresh=0.5, float hier=0.5);

    float* rgb2rgbpf(const cv::Mat &rgb);
}

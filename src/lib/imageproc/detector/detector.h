#ifndef __DETECTOR_H__
#define __DETECTOR_H__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "color_d.h"
#include "region_classifier.h"
#include "field_d.h"


namespace vision{
class Detector{
public:
    Detector(int width, int height);
    ~Detector(){
        free(lutCb);
        free(lutCr);
        free(buf);
        cudaFree(img_d);
    }
    void process(uint8_t *img_d);
    bool isGreen(int x, int y);
    const int* getBorder(){
        return fd->getConvexFieldBorder();
    }
    const std::vector<point_2d> getCorner() const {
        return fd->getCorner();
    }
private:
    int8_t * lutCb, * lutCr;
    int _width, _height;
    FieldColorDetector* fcd;
    RegionClassifier* rc;
    FieldDetector* fd;
    uint8_t *img_d, *buf;
};
}
#endif //DETECTOR_H

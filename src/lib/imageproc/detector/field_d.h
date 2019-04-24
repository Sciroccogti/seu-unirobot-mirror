#ifndef __FIELD_DETECTOR_H__
#define __FIELD_DETECTOR_H__

#include <random>
#include <vector>

#include "color_d.h"

#include "base_detector.h"
#include "line.h"
#include "region_classifier.h"

namespace vision {

#define MIN_DIS 20.0

struct Region {
    int xLeft, xRight, yTop, yBottom;
    int size;
    bool isField;
};

class FieldDetector : public BaseDetector {
private:
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist{0,1};
    std::vector<point_2d> borderPoints;

    int *fieldBorderFull;
    std::vector<point_2d> corner;

    FieldDetector() = delete;
    FieldDetector(const FieldDetector &cpy) = delete;
    FieldDetector operator=(FieldDetector &f) = delete;

public:
    FieldDetector(int width, int height, int8_t *lutCb, int8_t *lutCr) __attribute__((nonnull));
    ~FieldDetector();
    void proceed(const uint8_t *const img, const FieldColorDetector *const field, int spacing, Scanline* lines)
    __attribute__((nonnull));

    const int* getConvexFieldBorder() const {
        return fieldBorderFull;
    }

    const std::vector<point_2d> getCorner() const {
        return corner;
    }

};

}  // namespace vision
#endif  // __FIELD_DETECTOR_H__

#ifndef SEU_UNIROBOT_MATH_HPP
#define SEU_UNIROBOT_MATH_HPP

#include "matrix.hpp"
#include "vector.hpp"
#include "math_common.hpp"

namespace robot_math
{
    typedef vector_t<int, 3>    vector3i;
    typedef vector_t<float, 3>  vector3f;
    typedef vector_t<double, 3> vector3d;
    
    typedef vector_t<int, 2>    vector2i;
    typedef vector_t<float, 2>  vector2f;
    typedef vector_t<double, 2> vector2d;
}

#endif
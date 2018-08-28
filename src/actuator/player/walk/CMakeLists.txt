add_library(walk STATIC
            Spline.cpp
            CubicSpline.cpp
            Polynom.cpp
            WalkEngine.cpp)

target_link_libraries(walk pthread)
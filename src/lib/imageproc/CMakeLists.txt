add_subdirectory(detector)
add_library(imageproc
        imageproc.cpp)

target_link_libraries(imageproc
        darknet
        ${OpenCV_LIBS})

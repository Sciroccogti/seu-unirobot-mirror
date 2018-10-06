add_library(vision 
        vision.cpp)

target_link_libraries(vision 
        ${OpenCV_LIBS}
        imageproc
        darknet)

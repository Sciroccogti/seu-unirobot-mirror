add_library(vision STATIC
            vision.cpp
            capture.cpp)
            
            
target_link_libraries(vision
            ${OpenCV_LIBS}
            v4l2)
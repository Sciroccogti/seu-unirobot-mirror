add_library(vision STATIC
            vision.cpp
            camera.cpp)
            
            
target_link_libraries(vision
            ${OpenCV_LIBS}
            sensor
            pthread
            rt
            v4l2)
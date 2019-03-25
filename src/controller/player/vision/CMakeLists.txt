add_library(vision
        vision.cpp)

target_link_libraries(vision
        opencv_core
        opencv_imgcodecs
        opencv_imgproc
        cudaproc
        imageproc
        detector
        darknet
        server
        pthread
        rt
        stdc++
        cudart)

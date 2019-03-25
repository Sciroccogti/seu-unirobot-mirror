add_library(detector 
        detector.cu 
        color_d.cu 
        field_d.cu 
        region_classifier.cpp)
target_link_libraries(detector 
        cudart 
        ${OpenCV_LIBS})

cuda_add_library(imageproc
        image_process.cu
        image_process.cpp)

target_link_libraries(imageproc
        ${OpenCV_LIBS}
        ${CUDA_LIBRARIES})

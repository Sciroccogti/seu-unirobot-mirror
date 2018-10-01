file(GLOB CFILES "*.c")
file(GLOB CUFILES "*.cu")

cuda_add_library(darknet STATIC ${CUFILES} ${CFILES})
target_link_libraries(darknet 
    stdc++
    m
    cudnn
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY})

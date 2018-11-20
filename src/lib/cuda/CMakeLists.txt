cuda_add_library(cudaproc
        cudaproc.cu)

target_link_libraries(cudaproc
        ${CUDA_LIBRARIES})

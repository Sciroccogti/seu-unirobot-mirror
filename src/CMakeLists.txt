add_definitions(-DGPU)
add_definitions(-DCUDNN)
add_definitions(-DCUDNN_HALF)
#add_definitions(-DMVSDK)
find_package(OpenCV 3.3.1 REQUIRED)
find_package(CUDA REQUIRED)

################################################################################################
# A function for automatic detection of GPUs installed  (if autodetection is enabled)
# Usage:
#   detect_installed_gpus(out_variable)
function(detect_installed_gpus out_variable)
    if(NOT CUDA_gpu_detect_output)
        set(__cufile ${PROJECT_BINARY_DIR}/detect_cuda_archs.cu)

        file(WRITE ${__cufile} ""
                "#include <cstdio>\n"
                "int main()\n"
                "{\n"
                "  int count = 0;\n"
                "  if (cudaSuccess != cudaGetDeviceCount(&count)) return -1;\n"
                "  if (count == 0) return -1;\n"
                "  for (int device = 0; device < count; ++device)\n"
                "  {\n"
                "    cudaDeviceProp prop;\n"
                "    if (cudaSuccess == cudaGetDeviceProperties(&prop, device))\n"
                "      std::printf(\"%d%d \", prop.major, prop.minor);\n"
                "  }\n"
                "  return 0;\n"
                "}\n")

        execute_process(COMMAND "${CUDA_NVCC_EXECUTABLE}" "--run" "${__cufile}"
                WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/CMakeFiles/"
                RESULT_VARIABLE __nvcc_res OUTPUT_VARIABLE __nvcc_out
                ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

        if(__nvcc_res EQUAL 0)
            string(REPLACE "2.1" "2.1(2.0)" __nvcc_out "${__nvcc_out}")
            set(CUDA_gpu_detect_output ${__nvcc_out} CACHE INTERNAL "Returned GPU architetures from detect_gpus tool" FORCE)
        endif()
    endif()

    if(NOT CUDA_gpu_detect_output)
        message(STATUS "Automatic GPU detection failed. Building for all known architectures.")
        set(${out_variable} ${Caffe_known_gpu_archs} PARENT_SCOPE)
    else()
        set(${out_variable} ${CUDA_gpu_detect_output} PARENT_SCOPE)
    endif()
endfunction()

detect_installed_gpus(__cuda_arch_bin)

set(CMAKE_CUDA_FLAGS " -gencode arch=compute_${__cuda_arch_bin},code=[sm_${__cuda_arch_bin},compute_${__cuda_arch_bin}]")
message(STATUS "CUDA_FLAGS: ${CMAKE_CUDA_FLAGS}")
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")

include_directories(lib)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(lib/MVSDK/lib/x64)
add_subdirectory(lib)
add_subdirectory(controller)
if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

install(DIRECTORY data DESTINATION ${CMAKE_INSTALL_PREFIX})

file(GLOB PY_SCRIPTS "scripts/*.py")
install(PROGRAMS ${PY_SCRIPTS} DESTINATION ${CMAKE_INSTALL_PREFIX} 
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ GROUP_WRITE)

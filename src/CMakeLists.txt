include_directories(lib)
add_definitions(-DGPU)
add_definitions(-DCUDNN)
add_definitions(-DCUDNN_HALF)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -g -G -gencode=arch=compute_61,code=sm_61")
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")

add_subdirectory(lib)
add_subdirectory(controller)
if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

install(DIRECTORY data DESTINATION ${CMAKE_INSTALL_PREFIX})

file(GLOB PY_SCRIPTS "scripts/*.py")
install(PROGRAMS ${PY_SCRIPTS} DESTINATION ${CMAKE_INSTALL_PREFIX} 
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ GROUP_WRITE)

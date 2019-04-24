add_executable(auto_marker main.cpp)

target_link_libraries(auto_marker 
    boost_system 
    boost_filesystem 
    darknet
    cudaproc
    opencv_core
    opencv_imgcodecs
    opencv_imgproc)

install(TARGETS auto_marker RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_SYSTEM_PROCESSOR}")
include_directories(${Qt5Widgets_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

set(TARGET_NAME camera_calibration)

add_executable(${TARGET_NAME}
        main.cpp
        CameraDevice.cpp
        camera_calibration.cpp)

target_link_libraries(${TARGET_NAME}
        ${OpenCV_LIBS}
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
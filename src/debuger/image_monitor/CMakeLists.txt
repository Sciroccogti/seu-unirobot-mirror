include_directories(${Qt5Widgets_INCLUDE_DIRS})

set(TARGET_NAME image_monitor)

add_executable(${TARGET_NAME}
        main.cpp
        image_monitor.cpp)

target_link_libraries(${TARGET_NAME}
        ui
        tcp_client
        debuger_options
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
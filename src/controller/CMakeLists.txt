include_directories(./)
include_directories(./player)
add_subdirectory(options)
add_subdirectory(player)
add_subdirectory(dynamixel)

set(TARGET_NAME controller)

add_executable(${TARGET_NAME}
                main.cpp)

target_link_libraries(${TARGET_NAME}
                pthread
                v4l2
                controller_options
                player)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})

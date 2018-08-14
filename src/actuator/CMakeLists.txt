include_directories(./)

add_subdirectory(tcp_server)
add_subdirectory(options)
add_subdirectory(player)
add_subdirectory(dynamixel)

set(TARGET_NAME actuator)

add_executable(${TARGET_NAME}
                main.cpp)

target_link_libraries(${TARGET_NAME}
                tcp_server
                actuator_options
                player)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})

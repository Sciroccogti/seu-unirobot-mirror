include_directories(./)

add_subdirectory(communication)
add_subdirectory(options)
add_subdirectory(dynamixel)

set(TARGET_NAME actuator)

add_executable(${TARGET_NAME}
                main.cpp)

target_link_libraries(${TARGET_NAME}
                communication
                actuator_options)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
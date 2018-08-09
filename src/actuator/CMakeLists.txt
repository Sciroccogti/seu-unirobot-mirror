include_directories(./)

add_subdirectory(communication)
add_subdirectory(configuration)
add_subdirectory(dynamixel)

add_executable(actuator main.cpp)

target_link_libraries(actuator configuration)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(TARGETS actuator RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
include_directories(./lib)
add_subdirectory(actuator)
add_subdirectory(calculator)
add_subdirectory(debuger)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(DIRECTORY data DESTINATION ${CMAKE_INSTALL_PREFIX})

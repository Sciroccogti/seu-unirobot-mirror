include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(static_action
                main.cpp
                robot_gl.cpp
                joint_revise.cpp
                static_action.cpp)

target_link_libraries(static_action
                        glut
                        GLU
                        GL
                        Qt5::Widgets
                        Qt5::Core
                        Qt5::Gui)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
install(TARGETS static_action RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
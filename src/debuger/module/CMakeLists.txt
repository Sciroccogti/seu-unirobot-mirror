add_library(ui STATIC
            robot_gl.cpp)

target_link_libraries(ui
                        glut
                        GLU
                        GL
                        Qt5::Widgets
                        Qt5::Core
                        Qt5::Gui)
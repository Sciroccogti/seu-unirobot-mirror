include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_library(ui STATIC
            ImageLabel.cpp
            RobotGL.cpp)

target_link_libraries(ui
                        glut
                        GLU
                        GL
                        Qt5::Widgets
                        Qt5::Core
                        Qt5::Gui)
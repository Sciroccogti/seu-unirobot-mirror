include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_library(ui STATIC
            camera_setter.cpp
            joint_revise.cpp
            joint_setter.cpp
            walk_remote.cpp
            image_label.cpp
            state_monitor.cpp
            robot_gl.cpp)

target_link_libraries(ui
            robot
            opencv_core
            opencv_imgproc
            tcp_client
            glut
            GLU
            GL
            Qt5::Widgets
            Qt5::Core
            Qt5::Gui)
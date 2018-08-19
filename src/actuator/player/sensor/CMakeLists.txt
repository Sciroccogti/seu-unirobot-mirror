add_library(sensor STATIC
            capture.cpp
            debuger.cpp
            motor.cpp
            rmotor.cpp
            vmotor.cpp
            game_ctrl.cpp
            imu.cpp)
            
target_link_libraries(sensor
            boost_system
            dynamixel
            rt
            v4l2
            pthread)
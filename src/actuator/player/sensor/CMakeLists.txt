add_library(sensor STATIC
            motor.cpp
            game_ctrl.cpp
            imu.cpp
            remote.cpp)
            
target_link_libraries(sensor
            dynamixel
            tcp_server
            rt
            pthread)
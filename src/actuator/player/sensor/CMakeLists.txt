add_library(sensor STATIC
            debuger.cpp
            motor.cpp
            game_ctrl.cpp
            imu.cpp)
            
target_link_libraries(sensor
            boost_system
            dynamixel
            rt
            pthread)
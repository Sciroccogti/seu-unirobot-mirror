add_library(sensor STATIC
            dxl.cpp
            game_ctrl.cpp
            imu.cpp
            remote.cpp)
            
target_link_libraries(sensor
            dynamixel
            rt
            pthread)
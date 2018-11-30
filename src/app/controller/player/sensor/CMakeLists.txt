add_library(sensor STATIC
            camera.cpp
            motor.cpp
            imu.cpp)
            
target_link_libraries(sensor
            MVSDK
            server
            dynamixel
            boost_system)

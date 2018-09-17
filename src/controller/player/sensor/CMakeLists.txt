add_library(sensor STATIC
            camera.cpp
            motor.cpp
            server.cpp
            gamectrl.cpp
            hear.cpp
            imu.cpp)
            
target_link_libraries(sensor
            dynamixel
            boost_system
            v4l2
            rt
            pthread)
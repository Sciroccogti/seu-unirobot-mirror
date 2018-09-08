add_library(sensor STATIC
            motor.cpp
            server.cpp
            gamectrl.cpp
            hear.cpp
            imu.cpp)
            
target_link_libraries(sensor
            dynamixel
            boost_system
            rt
            pthread)
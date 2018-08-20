add_library(sensor STATIC
            server.cpp
            gamectrl.cpp
            imu.cpp)
            
target_link_libraries(sensor
            boost_system
            rt
            pthread)
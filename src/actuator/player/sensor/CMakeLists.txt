add_library(sensor STATIC
            tcp_server.cpp
            gamectrl.cpp
            hear.cpp
            imu.cpp)
            
target_link_libraries(sensor
            boost_system
            rt
            pthread)
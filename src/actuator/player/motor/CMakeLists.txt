add_library(motor STATIC
            motor.cpp
            vmotor.cpp
            rmotor.cpp)
            
target_link_libraries(motor
            dynamixel
            rt)
include_directories(.)

add_subdirectory(sensor)
add_subdirectory(motor)
add_subdirectory(vision)

add_library(player STATIC
            play_with_remote.cpp
            player.cpp)
            
target_link_libraries(player
            boost_system
            sensor
            motor
            vision
            rt)
include_directories(.)

add_subdirectory(sensor)

add_library(player STATIC
            play_with_remote.cpp
            player.cpp)
            
target_link_libraries(player
            sensor
            rt)
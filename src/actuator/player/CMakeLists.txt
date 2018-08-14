include_directories(.)

add_subdirectory(sensor)

add_library(player STATIC
            player.cpp)
            
target_link_libraries(player
            sensor
            rt)
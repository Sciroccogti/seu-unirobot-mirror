add_library(player STATIC
            player.cpp)
            
target_link_libraries(player
            sensor
            rt)
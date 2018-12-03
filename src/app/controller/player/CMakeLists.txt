add_subdirectory(sensor)
add_subdirectory(walk)
add_subdirectory(vision)
add_subdirectory(server)

add_library(player STATIC 
            play_with_remote.cpp
            player.cpp)
            
target_link_libraries(player
            walk
            server
            vision
            boost_system
            sensor)

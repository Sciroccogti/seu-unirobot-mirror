add_subdirectory(sensor)
add_subdirectory(motion)
add_subdirectory(vision)
add_subdirectory(server)

add_library(player STATIC 
            play_with_remote.cpp
            player.cpp)
            
target_link_libraries(player
            walk
            scan
            server
            vision
            boost_system
            sensor)

add_subdirectory(core)
add_subdirectory(sensor)
add_subdirectory(engine)
add_subdirectory(vision)
add_subdirectory(localization)
add_subdirectory(server)

add_library(player STATIC 
            play_with_remote.cpp
            play_without_gc.cpp
            play_with_gc.cpp
            skill.cpp
            player.cpp)
            
target_link_libraries(player
            core
            action
            walk
            scan
            led
            server
            vision
            boost_system
            sensor)

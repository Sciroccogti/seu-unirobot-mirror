include_directories(.)

add_subdirectory(sensor)
add_subdirectory(motor)
add_subdirectory(vision)
add_subdirectory(fsm)
add_subdirectory(walk)

add_library(player STATIC
            play_without_gamectrl.cpp
            play_with_gamectrl.cpp
            play_with_remote.cpp
            player.cpp)
            
target_link_libraries(player
            fsm
            walk
            boost_system
            sensor
            motor
            vision
            rt)

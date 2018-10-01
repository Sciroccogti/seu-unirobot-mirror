include_directories(.)

add_subdirectory(sensor)
add_subdirectory(fsm)
add_subdirectory(walk)
add_subdirectory(vision)

add_library(player STATIC
            play_without_gamectrl.cpp
            play_with_gamectrl.cpp
            play_with_remote.cpp
            player.cpp)
            
target_link_libraries(player
            ${OpenCV_LIBS}
            fsm
            walk
            vision
            boost_system
            sensor
            rt)

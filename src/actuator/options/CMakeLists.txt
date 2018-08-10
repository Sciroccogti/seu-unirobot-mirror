add_library(actuator_options STATIC
            options.cpp)
            
target_link_libraries(actuator_options
            boost_program_options)
add_library(configuration STATIC
            configuration.cpp)  
            
target_link_libraries(configuration
                        boost_program_options)
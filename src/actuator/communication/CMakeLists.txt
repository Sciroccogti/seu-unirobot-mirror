add_library(communication STATIC
            serial_port_handler.cpp
            tcp_server_handler.cpp)
            
target_link_libraries(communication
            boost_system
            pthread)
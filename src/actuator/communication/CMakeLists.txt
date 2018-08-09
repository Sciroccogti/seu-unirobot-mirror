add_library(communication STATIC
            serial_port_handler.cpp
            tcp_server_handler.hpp
            udp_server_handler.hpp)
            
target_link_libraries(communication boost_system pthread)
add_library(tcp_server STATIC
            tcp_server_handler.cpp)
            
target_link_libraries(tcp_server
            boost_system
            pthread)
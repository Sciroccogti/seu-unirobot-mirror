add_library(tcp_client STATIC
            tcp_client_handler.cpp)

target_link_libraries(tcp_client
            boost_system
            pthread)
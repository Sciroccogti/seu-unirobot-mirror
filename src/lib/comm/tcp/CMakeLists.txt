add_library(tcp STATIC
            tcp_server.hpp
            tcp_client.hpp
            tcp_packet.hpp)
            
target_link_libraries(tcp pthread)
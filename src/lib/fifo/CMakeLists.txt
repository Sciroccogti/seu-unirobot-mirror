add_library(fifo pipe_fifo.hpp shm_fifo.hpp)
target_link_libraries(fifo rt)
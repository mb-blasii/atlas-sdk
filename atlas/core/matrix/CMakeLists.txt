add_library(matrix INTERFACE
        mat4.h)

target_link_libraries(matrix
        INTERFACE
        vectors
)
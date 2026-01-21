add_library(quaternion INTERFACE
        quaternion.h
)

target_link_libraries(quaternion
        INTERFACE
        vectors
        math
)
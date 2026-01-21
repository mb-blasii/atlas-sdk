add_library(shapes INTERFACE
        shape2.h
        shape.h
)

target_link_libraries(shapes
        INTERFACE
        vectors
)




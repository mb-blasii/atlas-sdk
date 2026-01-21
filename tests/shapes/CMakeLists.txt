add_executable(shape2_tests
        shape2_test.cpp
        shape2_test.h
)

add_executable(shape_tests
        shape_test.cpp
        shape_test.h
)

target_include_directories(shape2_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_include_directories(shape_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)
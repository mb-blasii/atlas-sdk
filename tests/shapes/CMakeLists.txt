# Helper: copy atlas DLL next to test executable
function(atlas_copy_runtime target_name)
    add_custom_command(
            TARGET ${target_name}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:atlas>
            $<TARGET_FILE_DIR:${target_name}>
            COMMENT "Copying atlas runtime to ${target_name} output directory"
    )
endfunction()

# shape_tests
add_executable(shape_tests
        shape_test.cpp
        shape_test.h
)

target_link_libraries(shape_tests PRIVATE atlas)
atlas_copy_runtime(shape_tests)

# shape2_tests
add_executable(shape2d_tests
        shape2d_test.cpp
        shape2d_test.h
)

target_link_libraries(shape2d_tests PRIVATE atlas)
atlas_copy_runtime(shape2d_tests)

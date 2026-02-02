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
add_executable(transform_tests
        transform_test.cpp
        transform_test.h
)

target_link_libraries(transform_tests PRIVATE atlas)
atlas_copy_runtime(transform_tests)

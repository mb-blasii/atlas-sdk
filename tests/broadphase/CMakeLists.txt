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

# broadphase_tests
add_executable(broadphase_tests
        broadphase_test.cpp
        broadphase_test.h
)

target_link_libraries(broadphase_tests PRIVATE atlas)
atlas_copy_runtime(broadphase_tests)

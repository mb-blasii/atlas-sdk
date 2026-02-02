cmake_minimum_required(VERSION 3.15)

project(atlas
        VERSION 0.1.0
        LANGUAGES CXX
)

# =========================
# Global configuration
# =========================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =========================
# Atlas library
# =========================
file(GLOB_RECURSE ATLAS_SOURCES
        src/*.cpp
)

file(GLOB_RECURSE ATLAS_HEADERS
        include/*.h
)

add_library(atlas SHARED
        ${ATLAS_SOURCES}
        ${ATLAS_HEADERS}
)

target_include_directories(atlas
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_definitions(atlas
        PRIVATE ATLAS_EXPORTS
)

# =========================
# Install rules (SDK only)
# =========================
include(GNUInstallDirs)

install(TARGETS atlas
        EXPORT atlasTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}   # .dll
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}   # .so
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}   # .lib / .a
)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# =========================
# Export target (optional, future-proof)
# =========================
install(EXPORT atlasTargets
        FILE atlasTargets.cmake
        NAMESPACE atlas::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/atlas
)

# =========================
# Tests (DEV ONLY)
# =========================
option(ATLAS_ENABLE_TESTS "Enable Atlas tests (dev only)" ON)

if (ATLAS_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
